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

        // ── Build 8-byte request frame ────────────────────────────────────────────
        uint8_t tx[FRAME_SIZE] = {};
        tx[0] = cmd;
        tx[1] = seq_++;
        // tx[2..7] = 0x00 (reserved)

        if (!send_all(tx, FRAME_SIZE)) {
            close_socket();
            result.message = "TCP send failed: " + std::string(std::strerror(errno));
            continue;
        }

        return true;
    }
}

bool GripperTCPClient::try_recv(GripperResponse &result)
{
    uint8_t rx[FRAME_SIZE] = {};

    if (!recv_all(rx, FRAME_SIZE)) {   // blocks until full frame or timeout
        std::cout << "Fail in try_recv." << std::endl;
        return false;
    }

    // Same parsing as send_command
    result.status         = rx[4];
    result.actual_rpm     = (static_cast<uint16_t>(rx[5]) << 4) | ((rx[6] >> 4) & 0x0F);
    result.actual_current = (static_cast<uint16_t>((rx[6]) & 0x0F)) | rx[7];

    switch (result.status) {
        case STATUS_MOVING:
            result.success = true;
            result.message = "In progress";
            break;
        case STATUS_REACHED:
            result.success = true;
            result.message = "Object target reached";
            break;
        case STATUS_RETRACTED:
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

    configure_socket();  // <-- your idea

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port_);

    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
        printf("Inet fail.\n");
        close_socket();
        return false;
    }

    if (::connect(sock_fd_,
                  reinterpret_cast<struct sockaddr*>(&addr),
                  sizeof(addr)) < 0) {
        printf("Connect fail.\n");
        close_socket();
        return false;
    }

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
            close_socket();
            return false;
        }
        sent += static_cast<std::size_t>(n);
    }
    return true;
}

bool GripperTCPClient::recv_all(uint8_t * buf, std::size_t len)
{
    // SO_RCVTIMEO isn't set on the socket so ::recv() won't time out
    std::size_t received = 0;
    while (received < len) {
        ssize_t n = ::recv(sock_fd_,
                           buf + received,
                           len - received,
                           0);
        if (n == 0) {
            // Peer closed connection
            std::cerr << "Connection closed by peer\n";
            close_socket();
            return false;
        }

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available yet → NOT an error
                return false;  // or special status
            }

            std::cerr << "recv error: " << strerror(errno) << "\n";
            close_socket();
            return false;
        }   // timeout (EAGAIN) or closed connection

        received += static_cast<std::size_t>(n);
    }
    return true;
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