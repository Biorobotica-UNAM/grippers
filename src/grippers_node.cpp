#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "gripper_sdk.hpp"
#include "grippers_interfaces/srv/move_grippers.hpp"

using MoveGrippers = grippers_interfaces::srv::MoveGrippers;
using namespace std::chrono_literals;

namespace grippers
{

// =============================================================================
//  GripperNode
//
//  ROS 2 lifecycle:
//    - Reads RP2350 host/port from parameters at startup
//    - Opens a persistent TCP connection to the RP2350
//    - Advertises the /move_gripper service
//    - Each service call sends one 8-byte command and returns the response
//
//  Parameters:
//    rp2350_host    (string,  default "192.168.50.2")  RP2350 IP address
//    rp2350_port    (int,     default 7777)            RP2350 TCP port
//    tcp_timeout_ms (int,     default 1000)            per-call timeout in ms
// =============================================================================

class GripperNode : public rclcpp::Node
{
public:
    explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gripper_node", options)
    {
        // --- Declare parameters ---
        this->declare_parameter<std::string>("rp2350_host",    "192.168.50.2");
        this->declare_parameter<int>        ("rp2350_port",    8888);
        this->declare_parameter<int>        ("tcp_timeout_ms", 1000);

        const auto host       = this->get_parameter("rp2350_host").as_string();
        const auto port       = static_cast<uint16_t>(
                                    this->get_parameter("rp2350_port").as_int());
        const auto timeout_ms = static_cast<int>(
                                    this->get_parameter("tcp_timeout_ms").as_int());

        // --- Connect to RP2350 ---
        RCLCPP_INFO(this->get_logger(),
                    "Connecting to RP2350 at %s:%d (timeout %d ms)...",
                    host.c_str(), port, timeout_ms);
        try {
            tcp_client_ = std::make_unique<GripperTCPClient>(
                host, port, timeout_ms);
            RCLCPP_INFO(this->get_logger(), "TCP connection established.");
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(this->get_logger(),
                         "Failed to connect to RP2350: %s", e.what());
            throw;   // let the executor handle the fatal error
        }

        // --- Advertise service ---
        service_ = this->create_service<MoveGrippers>(
            "/move_grippers",
            std::bind(&GripperNode::handle_service, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "Service /move_grippers is ready.");
    }

private:
    // --- Service handler ---
    void handle_service(
        const std::shared_ptr<MoveGrippers::Request>  request,
        const std::shared_ptr<MoveGrippers::Response> response)
    {
        // Validate command byte
        if (request->cmd != CMD_OPEN &&
            request->cmd != CMD_CLOSE &&
            request->cmd != CMD_STOP &&
            request->cmd != CMD_QUERY_STATUS)
        {
            response->success = false;
            response->message = "Unknown cmd byte: " +
                                std::to_string(request->cmd) +
                                ". Use 0x01 (set speed) or 0x02 (query).";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Guard against a lost connection
        if (!tcp_client_ || !tcp_client_->is_connected()) {
            response->success = false;
            response->message = "TCP connection to RP2350 is not available";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Service call — cmd: 0x%02X", request->cmd);

        // Send command over TCP and wait for response
        const GripperResponse result =
            tcp_client_->send_command(request->cmd);

        // Fill ROS 2 response
        response->success    = result.success;
        response->status     = result.status;
        response->speed_rpm  = result.actual_rpm;
        response->current    = result.actual_current;
        response->message    = result.message;

        RCLCPP_INFO(this->get_logger(),
                    "Response — success: %s  status: 0x%02X  "
                    "actual_rpm: %u  current: %u  msg: %s",
                    result.success ? "true" : "false",
                    result.status,
                    result.actual_rpm,
                    result.actual_current,
                    result.message.c_str());
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::unique_ptr<GripperTCPClient>            tcp_client_;
    rclcpp::Service<MoveGrippers>::SharedPtr      service_;
};

}  // namespace gripper

// =============================================================================
//  main
// =============================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<grippers::GripperNode>());
    }
    catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"),
                     "Node terminated with exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}