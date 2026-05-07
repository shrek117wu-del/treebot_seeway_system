/**
 * @file teleop_watchdog_node.cpp
 * @brief ROS 2 Node for Ultra-Low Latency Teleoperation Watchdog
 * 
 * Subscribes to /teleop_joy (published by WebRTC Agent) and converts to 
 * TwistStamped commands for MoveIt2 Servo. Protects the robot by enforcing a 
 * 100ms network loss watchdog which zeros out commands if the connection drops.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <chrono>

using namespace std::chrono_literals;

class TeleopWatchdogNode : public rclcpp::Node {
public:
    TeleopWatchdogNode() : Node("teleop_watchdog_node"), connection_active_(false) {
        // Publishers and Subscribers
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
            
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/teleop_joy", 10,
            std::bind(&TeleopWatchdogNode::joy_callback, this, std::placeholders::_1));

        // Read scale parameters
        this->declare_parameter("linear_scale", 0.5);
        this->declare_parameter("angular_scale", 1.0);
        this->declare_parameter("watchdog_timeout_ms", 100);
        this->declare_parameter("ema_alpha", 0.4); // 0.0 (heavy smoothing) to 1.0 (no smoothing)

        // Initialize EMA state
        smoothed_linear_x_ = 0.0;
        smoothed_angular_z_ = 0.0;

        // Timer for watchdog
        watchdog_timer_ = this->create_wall_timer(
            50ms, std::bind(&TeleopWatchdogNode::watchdog_check, this));

        last_msg_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Teleop Watchdog Node initialized. Waiting for /teleop_joy...");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        last_msg_time_ = this->now();
        
        if (!connection_active_) {
            RCLCPP_INFO(this->get_logger(), "Teleop connection established!");
            connection_active_ = true;
        }

        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "frcobot_base_link";

        double lin_scale = this->get_parameter("linear_scale").as_double();
        double ang_scale = this->get_parameter("angular_scale").as_double();
        double alpha = this->get_parameter("ema_alpha").as_double();

        // Expected mapping from WebRTC JSON:
        // axes[0] = linear X
        // axes[2] = angular Z
        if (msg->axes.size() >= 3) {
            double target_linear_x = msg->axes[0] * lin_scale;
            double target_angular_z = msg->axes[2] * ang_scale;
            
            // Apply Exponential Moving Average (EMA)
            smoothed_linear_x_ = (alpha * target_linear_x) + ((1.0 - alpha) * smoothed_linear_x_);
            smoothed_angular_z_ = (alpha * target_angular_z) + ((1.0 - alpha) * smoothed_angular_z_);
            
            twist_msg.twist.linear.x = smoothed_linear_x_;
            twist_msg.twist.angular.z = smoothed_angular_z_;
        }

        twist_pub_->publish(twist_msg);
    }

    void watchdog_check() {
        if (!connection_active_) return;

        double timeout_ms = this->get_parameter("watchdog_timeout_ms").as_int();
        auto duration = this->now() - last_msg_time_;

        if (duration.seconds() * 1000.0 > timeout_ms) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "WebRTC Cloud Connection LOST / Delayed > %f ms! FIRING WATCHDOG (Zeroing velocities).", timeout_ms);
            
            // Publish zero twist to halt MoveIt2 Servo
            auto twist_msg = geometry_msgs::msg::TwistStamped();
            twist_msg.header.stamp = this->now();
            twist_msg.header.frame_id = "frcobot_base_link";
            twist_msg.twist.linear.x = 0.0;
            twist_msg.twist.linear.y = 0.0;
            twist_msg.twist.linear.z = 0.0;
            twist_msg.twist.angular.x = 0.0;
            twist_msg.twist.angular.y = 0.0;
            twist_msg.twist.angular.z = 0.0;
            
            twist_pub_->publish(twist_msg);
            
            // Reset EMA state so it doesn't jump when reconnected
            smoothed_linear_x_ = 0.0;
            smoothed_angular_z_ = 0.0;
            
            // Mark inactive until connection recovers
            connection_active_ = false;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    
    rclcpp::Time last_msg_time_;
    bool connection_active_;
    
    // EMA state variables
    double smoothed_linear_x_;
    double smoothed_angular_z_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopWatchdogNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
