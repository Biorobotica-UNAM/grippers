#include "gripper_sdk.hpp"

#include <stdexcept>
#include <cstring>
#include <cerrno>

// POSIX socket headers
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace grippers
{

// Maximum accepted payload size — protects against malformed packets or line noise
static constexpr uint16_t MAX_PAYLOAD = 256;

// =============================================================================
//  Construction / Destruction
// =============================================================================

GripperTCPClient::GripperTCPClient(const std::string & host,
                                   uint16_t port,
                                   int timeout_ms)
: host_(host), port_(port), timeout_ms_(timeout_ms) {
    if (connect_with_retry()) std::cout << "Gripper successfully connected." << std::endl;
}

GripperTCPClient::~GripperTCPClient() {
    close_socket();
}

// =============================================================================
//  Public API
// =============================================================================

bool GripperTCPClient::send_command(uint8_t cmd) {
    GripperResponse result;

    while(true) {
        if (!is_connected()) {
            connect_with_retry();
        }

        std::vector<uint8_t> payload;

        payload.push_back(cmd);

        uint16_t payload_len =
            static_cast<uint16_t>(payload.size());

        std::vector<uint8_t> tx;

        // Length prefix (Big Endian)
        tx.push_back((payload_len >> 8) & 0xFF);
        tx.push_back(payload_len & 0xFF);

        // Payload
        tx.insert(
            tx.end(),
            payload.begin(),
            payload.end());

        if (!send_all(tx.data(), tx.size())) {
            close_socket();
            result.message = "TCP send failed: " + std::string(std::strerror(errno));
            continue;
        }

        return true;
    }
}

bool GripperTCPClient::try_recv(GripperResponse &result)
{
    // 1. Read 2-byte header
    uint8_t header[2] = {0};

    RecvStatus status = recv_all(header, 2);

    if (status == RecvStatus::Timeout) {
        return false;   // no data yet, socket still valid
    }

    if (status == RecvStatus::Disconnected) {
        close_socket();
        connected_ = false;
        return false;
    }

    uint16_t payload_len = 
        (static_cast<uint16_t>(header[0]) << 8) | 
        header[1];

    // Reject invalid or oversized payloads before allocating or waiting
    if (payload_len == 0 || payload_len > MAX_PAYLOAD) {
        close_socket();
        connected_ = false;
        return false;
    }

    // 2. Read payload dynamically
    std::vector<uint8_t> rx(payload_len);
    status = recv_all(rx.data(), payload_len);

    if (status == RecvStatus::Timeout) {
        return false;
    }

    if (status == RecvStatus::Disconnected) {
        close_socket();
        connected_ = false;
        return false;
    }

    // 3. Parse payload
    if (payload_len < 9)
    {
        return false;
    }

    // ---------------------------------
    // Deserialize motor report
    // ---------------------------------
    result.actual_rpm =
        (static_cast<uint16_t>(rx[0]) << 8) |
         static_cast<uint16_t>(rx[1]);

    result.actual_current =
        (static_cast<uint16_t>(rx[2]) << 8) |
         static_cast<uint16_t>(rx[3]);

    result.position =
        (static_cast<int32_t>(rx[4]) << 24) |
        (static_cast<int32_t>(rx[5]) << 16) |
        (static_cast<int32_t>(rx[6]) << 8)  |
         static_cast<int32_t>(rx[7]);

    result.status = rx[8];

    switch (result.status) {
        case STATUS_OPENING:
            result.success = true;
            result.message = "In progress";
            break;
        case STATUS_CLOSING:
            result.success = true;
            result.message = "In progress";
            break;
        case STATUS_REACHED:
            result.success = true;
            result.message = "Object target reached";
            break;
        case STATUS_IDLE:
            result.success = true;
            result.message = "Gripper retracted to initial position";
            break;
        case STATUS_STALLED:
            result.success = false;
            result.message = "Motor stalled — could not reach target";
            break;
        default:
            result.success = false;
            result.message = "Unknown status: " + std::to_string(result.status);
            break;
    }

    return true;
}

bool GripperTCPClient::connect_with_retry()
{
    while (true) {
        if (connect_once()) {
            std::cout << "Connected!" << std::endl;
            return true;
        }

        std::cerr << "Connection failed. Retrying in 1s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// =============================================================================
//  Private helpers
// =============================================================================

bool GripperTCPClient::connect_once()
{
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;
    printf("Socket created successfully.\n");

    // Poner no bloqueante ANTES de connect() — así connect() no se cuelga
    // en el timeout del kernel (~127s en Linux por default).
    int orig_flags = ::fcntl(sock_fd_, F_GETFL, 0);
    ::fcntl(sock_fd_, F_SETFL, orig_flags | O_NONBLOCK);

    configure_socket();

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port_);

    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
        printf("Inet fail.\n");
        close_socket();
        return false;
    }

    int rc = ::connect(sock_fd_,
                       reinterpret_cast<struct sockaddr*>(&addr),
                       sizeof(addr));

    if (rc < 0 && errno != EINPROGRESS) {
        // Falla inmediata: red inalcanzable, host rechaza, etc.
        printf("Connect fail (immediate): %s\n", strerror(errno));
        close_socket();
        connected_ = false;
        return false;
    }

    if (rc < 0) {
        // EINPROGRESS: el handshake quedó en curso. Esperamos a que el fd
        // se vuelva escribible, con timeout real controlado por nosotros.
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(sock_fd_, &wfds);

        struct timeval tv;
        tv.tv_sec  = timeout_ms_ / 1000;
        tv.tv_usec = (timeout_ms_ % 1000) * 1000;

        int sel = ::select(sock_fd_ + 1, nullptr, &wfds, nullptr, &tv);

        if (sel == 0) {
            printf("Connect timed out after %d ms.\n", timeout_ms_);
            close_socket();
            connected_ = false;
            return false;
        }
        if (sel < 0) {
            printf("select() error: %s\n", strerror(errno));
            close_socket();
            connected_ = false;
            return false;
        }

        // El fd quedó escribible — eso no garantiza éxito, hay que
        // chequear SO_ERROR para distinguir "conectó" de "falló async".
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        if (::getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 ||
            so_error != 0)
        {
            printf("Connect fail (deferred): %s\n",
                   so_error ? strerror(so_error) : "getsockopt failed");
            close_socket();
            connected_ = false;
            return false;
        }
    }

    // Listo: la conexión está armada. Volvemos a modo bloqueante porque
    // send_all()/recv_all() dependen de SO_SNDTIMEO/SO_RCVTIMEO, que solo
    // tienen efecto sobre sockets bloqueantes.
    ::fcntl(sock_fd_, F_SETFL, orig_flags);

    connected_ = true;
    return true;
}

void GripperTCPClient::configure_socket()
{
    int flag = 1;
    ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY,
                 &flag, sizeof(flag));

    struct timeval tv;
    tv.tv_sec  = timeout_ms_ / 1000;
    tv.tv_usec = (timeout_ms_ % 1000) * 1000;

    ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

bool GripperTCPClient::send_all(const uint8_t * buf, std::size_t len)
{
    std::size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(sock_fd_,
                           buf + sent,
                           len - sent,
                           MSG_NOSIGNAL);   // prevent SIGPIPE on broken pipe
        if (n <= 0) {
            connected_ = false;
            close_socket();
            return false;
        }
        sent += static_cast<std::size_t>(n);
    }
    return true;
}

GripperTCPClient::RecvStatus
GripperTCPClient::recv_all(uint8_t * buf, std::size_t len)
{
    std::size_t received = 0;
    while (received < len) {
        ssize_t n = ::recv(sock_fd_, buf + received, len - received, 0);
        if (n == 0) {
            connected_ = false;
            close_socket();
            return RecvStatus::Disconnected;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return RecvStatus::Timeout;     // socket is fine, just no data yet
            }
            std::cerr << "recv error: " << strerror(errno) << "\n";
            connected_ = false;
            close_socket();
            return RecvStatus::Disconnected;
        }
        received += static_cast<std::size_t>(n);
    }
    return RecvStatus::Ok;
}

void GripperTCPClient::close_socket()
{
    if (sock_fd_ >= 0) {
        ::shutdown(sock_fd_, SHUT_RDWR);
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

}  // namespace gripper