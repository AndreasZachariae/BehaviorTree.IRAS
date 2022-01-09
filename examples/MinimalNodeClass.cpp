/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : Examples
 * Purpose : Example of a minimal ROS2-Node class 
 *           which inherits from rclcpp::Node
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cpp_core/default.h>

class MinimalNodeClass : public rclcpp::Node
{
public:
    MinimalNodeClass();

private:
    bool stop_recieved_ = false;
    unsigned char diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;
};

MinimalNodeClass::MinimalNodeClass() : Node("MinimalNodeClass")
{
    stop_subscription_ = create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr) {
        stop_recieved_ = true;
        RCLCPP_WARN(get_logger(), "Stop recieved, resetting...");
    });

    diagnostic_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), [&]() {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic.level = diagnostic_status_;
        diagnostic.name = "MinimalHardware";
        diagnostic.hardware_id = "1";

        diagnostic_status_publisher_->publish(diagnostic);
    });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalNodeClass>());
    rclcpp::shutdown();

    return 0;
}