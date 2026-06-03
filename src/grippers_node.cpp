#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rclcpp/rclcpp.hpp"
#include "gripper_sdk.hpp"
#include "grippers_interfaces/srv/move_grippers.hpp"

#define CNT_TO_RAD 0.000125f

using MoveGrippers = grippers_interfaces::srv::MoveGrippers;
using namespace std::chrono_literals;

namespace grippers
{

class GripperNode : public rclcpp::Node
{
public:
    explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gripper_node", options)
    {
        this->declare_parameter<bool>                         ("simulation",     false);  // NEW
        this->declare_parameter<std::string>                  ("ip",             "172.27.1.120");
        this->declare_parameter<int>                          ("port",           8888);
        this->declare_parameter<int>                          ("tcp_timeout_ms", 1000);
        this->declare_parameter<std::string>                  ("joint_states_topic",         "joint_states");
        this->declare_parameter<std::string>                  ("gripper_controller_service", "move_gripper");
        this->declare_parameter<std::vector<std::string>>     ("joint_names", std::vector<std::string>());

        simulation_ = this->get_parameter("simulation").as_bool();  // NEW
        host        = this->get_parameter("ip").as_string();
        port        = static_cast<uint16_t>(this->get_parameter("port").as_int());
        timeout_ms  = static_cast<int>(this->get_parameter("tcp_timeout_ms").as_int());

        joint_states_topic = this->get_parameter("joint_states_topic").as_string();
        service_name       = this->get_parameter("gripper_controller_service").as_string();
        joint_names        = this->get_parameter("joint_names").as_string_array();

        // --- Connect to hardware only when not simulating ---
        if (!simulation_)  // NEW BLOCK
        {
            RCLCPP_INFO(this->get_logger(),
                        "Connecting to RP2350 at %s:%d (timeout %d ms)...",
                        host.c_str(), port, timeout_ms);
            try {
                tcp_client_ = std::make_unique<GripperTCPClient>(host, port, timeout_ms);
                RCLCPP_INFO(this->get_logger(), "TCP connection established.");
            }
            catch (const std::exception & e) {
                RCLCPP_FATAL(this->get_logger(),
                             "Failed to connect to RP2350: %s", e.what());
                throw;
            }

            // Start reader thread only for real hardware
            running_ = true;
            reader_thread_ = std::thread(&GripperNode::reader_loop, this);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Running in SIMULATION mode");
            running_ = false;
        }

        initialize_messages();

        service_ = this->create_service<MoveGrippers>(
            service_name,
            std::bind(&GripperNode::handle_service, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            joint_states_topic, 10);

        timer = this->create_wall_timer(
            100ms, std::bind(&GripperNode::timer_callback, this));

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

        positions.push_back(0.0);
        positions.push_back(0.0);
    }

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

            GripperResponse pkt;
            if (!tcp_client_->try_recv(pkt)) continue;

            if (pkt.status != STATUS_OPENING || pkt.status != STATUS_CLOSING) {
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

    void handle_service(
        const std::shared_ptr<MoveGrippers::Request>  request,
        const std::shared_ptr<MoveGrippers::Response> response)
    {
        // --- Simulation: resolve command and fake a response ---
        if (simulation_)  // NEW BLOCK
        {
            uint8_t cmd;
            if      (request->cmd == "open")   cmd = CMD_OPEN;
            else if (request->cmd == "close")  cmd = CMD_CLOSE;
            else if (request->cmd == "stop")   cmd = CMD_STOP;
            else if (request->cmd == "status") cmd = CMD_QUERY_STATUS;
            else {
                response->success = false;
                response->message = "Unknown cmd: " + request->cmd;
                return;
            }

            // Drive simulated position
            if      (cmd == CMD_OPEN)  { for (auto & p : positions) p =  0.7853; }
            else if (cmd == CMD_CLOSE) { for (auto & p : positions) p =  0.0;    }
            // CMD_STOP / CMD_QUERY_STATUS leave positions unchanged

            response->success   = true;
            response->status    = "REACHED";
            response->message   = "Goal reached (simulation)";
            response->speed_rpm = 0;
            response->position  = static_cast<int32_t>(
                (0.7853 - positions[0]) / CNT_TO_RAD);  // inverse of update_positions
            response->current   = 0;

            RCLCPP_INFO(this->get_logger(),
                        "Simulation — cmd: %s  position: %.4f rad",
                        request->cmd.c_str(), positions[0]);
            return;
        }

        // --- Real hardware path (unchanged) ---
        static uint8_t     cmd;
        static std::string status;

        if      (request->cmd == "open")   cmd = CMD_OPEN;
        else if (request->cmd == "close")  cmd = CMD_CLOSE;
        else if (request->cmd == "stop")   cmd = CMD_STOP;
        else if (request->cmd == "status") cmd = CMD_QUERY_STATUS;
        else {
            response->success = false;
            response->message = "Unknown cmd byte: " + request->cmd +
                                ". Use open / close / stop / status.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        if (!tcp_client_ || !tcp_client_->is_connected()) {
            RCLCPP_WARN(this->get_logger(), "Reconnecting TCP...");
            if (!tcp_client_->connect_with_retry()) {
                response->success = false;
                response->message = "Failed to reconnect";
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_ready_         = false;
            waiting_for_response_ = true;
        }

        RCLCPP_INFO(this->get_logger(), "Service call — cmd: %s", request->cmd.c_str());

        if (!tcp_client_->send_command(cmd))
            RCLCPP_ERROR(this->get_logger(), "TCP Client couldn't send the command.");

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

        update_positions(result);
        fill_response(result, response);

        RCLCPP_INFO(this->get_logger(),
                    "Response — success: %s  status: %s  "
                    "actual_rpm: %u  position: %d  current: %u  msg: %s",
                    result.success ? "true" : "false",
                    status.c_str(),
                    result.actual_rpm,
                    result.position,
                    result.actual_current,
                    result.message.c_str());
    }

    void timer_callback()
    {
        joint_states_.header.stamp = this->now();
        joint_states_.position     = positions;
        joint_states_pub_->publish(joint_states_);
    }

    void fill_response(const GripperResponse & result,
                       const std::shared_ptr<MoveGrippers::Response> & response)
    {
        static const std::unordered_map<uint8_t, std::string> status_map = {
            { STATUS_IDLE,    "IDLE"    },
            { STATUS_REACHED, "REACHED" },
            { STATUS_STALLED, "STALLED" },
        };

        auto it = status_map.find(result.status);
        response->status  = (it != status_map.end()) ? it->second : "UNKNOWN";
        response->success = (result.status == STATUS_REACHED ||
                             result.status == STATUS_IDLE);
        response->message = result.message;
        response->speed_rpm = result.actual_rpm;
        response->position  = result.position;
        response->current   = result.actual_current;
    }

    void update_positions(const GripperResponse & result)
    {
        double position_rad = 0.7853 - CNT_TO_RAD * result.position;
        for (auto & pos : positions) pos = position_rad;
    }

    // Members
    rclcpp::TimerBase::SharedPtr                               timer;
    std::unique_ptr<GripperTCPClient>                          tcp_client_;
    rclcpp::Service<MoveGrippers>::SharedPtr                   service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    sensor_msgs::msg::JointState                               joint_states_;

    // Params
    bool        simulation_ = false;  // NEW
    std::string host;
    uint16_t    port;
    int         timeout_ms;

    std::string joint_states_topic;
    std::string service_name;

    std::vector<std::string> joint_names;
    std::vector<double>      positions;

    // Reader thread
    std::thread       reader_thread_;
    std::atomic<bool> running_;

    // Service ↔ reader handoff
    std::mutex              result_mutex_;
    std::condition_variable result_cv_;
    bool                    result_ready_         = false;
    bool                    waiting_for_response_ = false;
    GripperResponse         latest_result_;
};

}  // namespace grippers

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