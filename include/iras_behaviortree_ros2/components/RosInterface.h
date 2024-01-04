/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Creates a ROS2 Node
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <iras_behaviortree_ros2/default.h>

#include <rclcpp/rclcpp.hpp>

#include <iras_behaviortree_ros2/core/Component.h>

class RosInterface : public Component
{
public:
    static std::string node_name;

    static rclcpp::Node::SharedPtr get_node_handle();

    static void init(std::string node_name, int argc, char **argv);
    static void init(std::string node_name = "Coordinator") { init(node_name, 0, nullptr); }

    static void spin_some() { rclcpp::spin_some(get_node_handle()); }

    static void shutdown()
    {
        get_node_handle()->~Node();
        rclcpp::shutdown();
    }

    RosInterface(const std::string &name);
    ~RosInterface();

protected:
    // Maybe create class NodeInfo with packages, name, ... instead of only names
    std::vector<std::string> required_nodes;

    void check_required_nodes();

private:
    static rclcpp::Node::SharedPtr node_handle_;

    static uint instance_count_;
};