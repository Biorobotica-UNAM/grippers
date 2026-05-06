#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <sensor_msgs/msg/joint_state.hpp>

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
//    rp2350_host    (string,  default "172.27.1.25")  RP2350 IP address
//    rp2350_port    (int,     default 8888)            RP2350 TCP port
//    tcp_timeout_ms (int,     default 1000)            per-call timeout in ms
// =============================================================================

class GripperNode : public rclcpp::Node
{
public:
    explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gripper_node", options)
    {
        // --- Declare parameters ---
        this->declare_parameter<std::string>              ("ip",             "172.27.1.25");
        this->declare_parameter<int>                      ("port",           8888);
        this->declare_parameter<int>                      ("tcp_timeout_ms", 1000);

        this->declare_parameter<std::string>              ("joint_states_topic",         "joint_states");
        this->declare_parameter<std::string>              ("gripper_controller_service", "move_gripper");

        this->declare_parameter<std::vector<std::string>> ("joint_names", std::vector<std::string>());

        host       = this->get_parameter("ip").as_string();
        port       = static_cast<uint16_t>(
                                    this->get_parameter("port").as_int());
        timeout_ms = static_cast<int>(
                                    this->get_parameter("tcp_timeout_ms").as_int());

        joint_states_topic  = this->get_parameter("joint_states_topic").as_string();
        service_name        = this->get_parameter("gripper_controller_service").as_string();

        joint_names = this->get_parameter("joint_names").as_string_array();

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

        initialize_messages();

        // --- Advertise service ---
        service_ = this->create_service<MoveGrippers>(
            service_name,
            std::bind(&GripperNode::handle_service, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);

        timer = this->create_wall_timer(
            100ms, std::bind(&GripperNode::timer_callback, this));

        // --- Start reader thread ---
        running_ = true;
        reader_thread_ = std::thread(&GripperNode::reader_loop, this);

        RCLCPP_INFO(this->get_logger(),
                    ("Service /" + service_name + " is ready.").c_str());
    }

    ~GripperNode()
    {
        running_ = false;
        if (reader_thread_.joinable()) reader_thread_.join();
    }

private:
    void initialize_messages()
    {
        joint_states_.name     = joint_names;
        joint_states_.position.resize(joint_names.size(), 0.0);
        joint_states_.velocity.resize(joint_names.size(), 0.0);
        joint_states_.effort.resize(joint_names.size(),   0.0);

        dummie_positions.push_back(0.0);
        dummie_positions.push_back(0.0);
    }

    // blocks on TCP recv, dispatches progress vs final
    void reader_loop()
    {
        while (running_) {
            if (!tcp_client_ || !tcp_client_->is_connected()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            if (!waiting_for_response_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Blocks until any packet arrives (progress or final)
            GripperResponse pkt;
            if (!tcp_client_->try_recv(pkt)) {   // non-throwing, returns false on timeout
                continue;
            }

            if (pkt.status != STATUS_MOVING) {
                // Terminal packet — wake up the service handler
                {
                    std::lock_guard<std::mutex> lock(result_mutex_);
                    latest_result_ = pkt;
                    result_ready_  = true;
                    waiting_for_response_ = false;
                }
                result_cv_.notify_one();
            }
        }
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
            RCLCPP_WARN(this->get_logger(), "Reconnecting TCP...");

            if (!tcp_client_->connect_with_retry()) {
                response->success = false;
                response->message = "Failed to reconnect";
                return;
            }
        }

        // Reset result flag before sending
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_ready_ = false;
            waiting_for_response_ = true;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Service call — cmd: %s", request->cmd.c_str());

        if (!tcp_client_->send_command(cmd)) RCLCPP_ERROR(this->get_logger(),
                                                            "TCP Client couldn't send the command.");

        // Wait for reader thread to deliver a terminal packet
        GripperResponse result;
        {
            std::unique_lock<std::mutex> lock(result_mutex_);
            if (!result_cv_.wait_for(lock, std::chrono::seconds(3),
                                [this] { return result_ready_; }))
            {
                response->success = false;
                response->message = "Timeout waiting for response";
                return;
            }

            result = latest_result_;
        }

        fill_response(result, response);

        RCLCPP_INFO(this->get_logger(),
                    "Response — success: %s  status: %s  "
                    "actual_rpm: %u  current: %u  msg: %s",
                    result.success ? "true" : "false",
                    status.c_str(),
                    result.actual_rpm,
                    result.actual_current,
                    result.message.c_str());
    }

    void timer_callback() {
        joint_states_.header.stamp = this->now();
        joint_states_.position     = dummie_positions;

        joint_states_pub_->publish(joint_states_);
    }

    void fill_response(const GripperResponse & result,
                       const std::shared_ptr<MoveGrippers::Response> & response)
    {
        static const std::unordered_map<uint8_t, std::string> status_map = {
            { STATUS_RETRACTED,   "RETRACTED"   },
            { STATUS_REACHED,     "REACHED"     },
            { STATUS_STALLED,     "STALLED"     },
        };

        auto it = status_map.find(result.status);
        response->status  = (it != status_map.end()) ? it->second : "UNKNOWN";
        response->success = (result.status == STATUS_REACHED ||
                             result.status == STATUS_RETRACTED);
        response->message = result.message;
        response->speed_rpm = result.actual_rpm;
        response->current   = result.actual_current;
    }


    // Members
    rclcpp::TimerBase::SharedPtr                               timer;
    std::unique_ptr<GripperTCPClient>                          tcp_client_;
    rclcpp::Service<MoveGrippers>::SharedPtr                   service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    sensor_msgs::msg::JointState                               joint_states_;

    // Params
    std::string host;
    uint16_t port;
    int timeout_ms;

    std::string joint_states_topic;
    std::string service_name;

    std::vector<std::string> joint_names;
    std::vector<double> dummie_positions;

    // Reader thread
    std::thread          reader_thread_;
    std::atomic<bool>    running_;

    // Service ↔ reader handoff
    std::mutex              result_mutex_;
    std::condition_variable result_cv_;
    bool                    result_ready_ = false;
    bool                    waiting_for_response_ = false;      
    GripperResponse         latest_result_;
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