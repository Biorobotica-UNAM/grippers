#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "gripper_sdk.hpp"
#include "grippers_interfaces/srv/move_grippers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
        declare_and_get_parameters();

        init_joint_states();

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
            service_name,
            std::bind(&GripperNode::handle_service, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        pub_joint_state = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "Service %s is ready.", service_name);

        timer = this->create_wall_timer(80ms, std::bind(&GripperNode::timer_callback, this));
    }

private:
    // --- Declare parameters ---
    void declare_and_get_parameters(void) {
        this->declare_parameter<std::string>("gripper_ip",     "172.27.1.120");
        this->declare_parameter<int>        ("gripper_port",   8888);
        this->declare_parameter<int>        ("tcp_timeout_ms", 1000);

        this->declare_parameter<std::string>("joint_states_topic",         "gripper/joint_states");
        this->declare_parameter<std::string>("gripper_controller_service", "move_gripper");

        this->declare_parameter("joint_names", std::vector<std::string>{});

        host       = this->get_parameter("gripper_ip").as_string();
        port       = static_cast<uint16_t>(
                                    this->get_parameter("gripper_port").as_int());
        timeout_ms = static_cast<int>(
                                    this->get_parameter("tcp_timeout_ms").as_int());

        joint_states_topic = this->get_parameter("joint_states_topic").as_string();
        service_name = this->get_parameter("gripper_controller_service").as_string();

        joint_names = this->get_parameter("joint_names").as_string_array();
    }

    void init_joint_states(void) {
        joint_states.name = joint_names;
        joint_states.header.stamp = this->now();
        joint_states.position = {0.78539, 0.78539};

        pub_joint_state->publish(joint_states);
    }

    // --- Service handler ---
    void handle_service(
        const std::shared_ptr<MoveGrippers::Request>  request,
        const std::shared_ptr<MoveGrippers::Response> response)
    {
        static uint8_t     cmd;
        static std::string status;

        // Validate command byte
        if (request->cmd == "open") cmd = CMD_OPEN;
        else if (request->cmd == "close") cmd = CMD_CLOSE;
        else if (request->cmd == "stop") cmd = CMD_STOP;
        else if (request->cmd == "status") cmd = CMD_QUERY_STATUS;
        else {
            response->success = false;
            response->message = "Unknown cmd byte: " +
                                request->cmd +
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
                    "Service call — cmd: %s", request->cmd.c_str());

        // Send command over TCP and wait for response
        result = tcp_client_->send_command(cmd);

        if(result.status == 0x01) status = "OPENING";
        if(result.status == 0x02) status = "CLOSING";
        if(result.status == 0x03) status = "REACHED";
        if(result.status == 0x04) status = "IDLE";
        if(result.status == 0x05) status = "STALLED";

        // Fill ROS 2 response
        response->success    = result.success;
        response->status     = status;
        response->position   = result.actual_count;
        response->current    = result.actual_current;
        response->message    = result.message;

        RCLCPP_INFO(this->get_logger(),
                    "Response — success: %s  status: %s  "
                    "actual_rpm: %u  current: %u  msg: %s",
                    result.success ? "true" : "false",
                    status.c_str(),
                    result.actual_count,
                    result.actual_current,
                    result.message.c_str());
    }

    std::vector<double> count_to_radians(int32_t count) {
        std::vector<double> joint_positions(2);
        float position = 6.2832 * count / (40 * 26 * 48);

        for (uint8_t i = 0; i < 2; i++) joint_positions[i] = position;

        return joint_positions;
    }

    void timer_callback(void) {
        joint_states.header.stamp = this->now();
        joint_states.position = count_to_radians(result.actual_count);

        pub_joint_state->publish(joint_states);
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    std::string host;
    uint16_t port;
    int timeout_ms;

    std::string joint_states_topic;
    std::string service_name;

    std::vector<std::string> joint_names;

    // ── Parameters ────────────────────────────────────────────────────────────
    sensor_msgs::msg::JointState joint_states;

    // ── TCP Response ────────────────────────────────────────────────────────────
    GripperResponse result;

    // ── Members ───────────────────────────────────────────────────────────────
    std::unique_ptr<GripperTCPClient>            tcp_client_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
    rclcpp::Service<MoveGrippers>::SharedPtr      service_;

    rclcpp::TimerBase::SharedPtr timer;
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