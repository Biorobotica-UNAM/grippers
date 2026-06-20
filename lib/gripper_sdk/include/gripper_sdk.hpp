#pragma once

#include <string>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

// =============================================================================
//  GripperSDK
//
//  Manages a single persistent TCP connection to the RP2350 + W5500.
//  The RP2350 acts as a TCP server (LISTEN state).
//  This class is the client — it connects once at construction time and
//  keeps the socket open for the lifetime of the ROS 2 node.
//
//  Wire protocol (fixed 8-byte frames, both directions):
//
//  Request  (PC → RP2350):
//    [0] cmd       0x01=open  0x02=close 0x03=stop 0x04=query_status
//    [1] seq       sequence number echoed in response
//    [2..7]        reserved (0x00)
//
//  Response (RP2350 → PC):
//    [0] cmd       echo of request cmd
//    [1] seq       echo of request seq
//    [2] status    0x01 moving | 0x02 reached | 0x03 stalled | 0x04 idle
//    [3] rpm_hi    actual RPM high byte
//    [4] rpm_lo    actual RPM low  byte
//    [5] curr_hi   actual current high byte
//    [6] curr_lo   actual current low byte
// =============================================================================

namespace grippers
{

constexpr uint8_t CMD_OPEN         = 0x01;
constexpr uint8_t CMD_CLOSE        = 0x02;
constexpr uint8_t CMD_STOP         = 0x03;
constexpr uint8_t CMD_QUERY_STATUS = 0x04;

constexpr uint8_t STATUS_OPENING    = 0x01;
constexpr uint8_t STATUS_CLOSING    = 0x02;
constexpr uint8_t STATUS_REACHED    = 0x03;
constexpr uint8_t STATUS_IDLE       = 0x04;
constexpr uint8_t STATUS_STALLED    = 0x05;

constexpr std::size_t FRAME_SIZE   = 8;

struct GripperResponse
{
    bool     success        {false};
    uint8_t  status         {0};
    int32_t  position       {0};
    uint16_t actual_rpm     {0};
    uint16_t actual_current {0};
    std::string message;
};

class GripperTCPClient
{
public:
    // Connect to the RP2350 at construction.
    // Throws std::runtime_error if the connection cannot be established.
    GripperTCPClient(const std::string & host, uint16_t port,
                     int timeout_ms = 1000);

    ~GripperTCPClient();

    // Not copyable — owns a socket fd
    GripperTCPClient(const GripperTCPClient &)             = delete;
    GripperTCPClient & operator=(const GripperTCPClient &) = delete;

    // Send a command and wait for the response.
    // Returns a GripperResponse describing the outcome.
    // On TCP error the socket is marked as disconnected and success=false.
    bool send_command(uint8_t cmd);
    bool try_recv(GripperResponse &result);

    bool is_connected() const { return connected_; }

    bool connect_with_retry();

    enum class RecvStatus { Ok, Timeout, Disconnected };

private:
    bool connect_once();

    void configure_socket();

    // Send exactly `len` bytes — handles EINTR/partial writes
    bool send_all(const uint8_t * buf, std::size_t len);

    // Receive exactly `len` bytes within timeout_ms_ — handles partial reads
    RecvStatus recv_all(uint8_t * buf, std::size_t len);

    void close_socket();

    int         sock_fd_    {-1};
    std::string host_;
    uint16_t    port_;
    int         timeout_ms_ {1000};
    bool        connected_{false};
    uint8_t     seq_        {0};    // rolling sequence number
    uint8_t     last_seq_   {0};
};

}  // namespace gripper