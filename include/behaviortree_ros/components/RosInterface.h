/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : behaviortree_ros
 * Purpose : Creates a ROS2 Node
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <rclcpp/rclcpp.hpp>

#include <cpp_core/Component.h>

class RosInterface : public Component
{
public:
    static std::string node_name;

    static rclcpp::Node::SharedPtr get_node_handle();

    static void init(std::string node_name, int argc, char **argv);
    static void init(std::string node_name = "TaskPlanner") { init(node_name, 0, nullptr); }

    static void spin_some() { rclcpp::spin_some(get_node_handle()); }

    static void shutdown() { rclcpp::shutdown(); }

    RosInterface(const std::string &name);
    ~RosInterface();

    virtual std::string ros_name() { return name_; }

protected:
    // Maybe create class NodeInfo with packages, name, ... instead of only names
    std::vector<std::string> required_nodes;

    void check_required_nodes();

private:
    static rclcpp::Node::SharedPtr node_handle_;

    static uint instance_count_;
};