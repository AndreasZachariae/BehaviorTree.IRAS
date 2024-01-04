/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Abstraction for handling ports and their values
 *
 * @author Andreas Zachariae
 * @since 2.0.0 (2023.12.13)
 *********************************************************/
#pragma once

#include <iras_behaviortree_ros2/default.h>

#include <behaviortree_cpp_v3/tree_node.h>

class PortHandler
{
public:
    PortHandler(BT::TreeNode *node) : bt_node_handle_(node) {}

    template <typename T>
    auto get_value(std::string name) const
    {
        BT::Optional<T> input = bt_node_handle_->getInput<T>(name);
        if (!input.has_value())
        {
            throw BT::RuntimeError("missing required input [" + name + "]: ", input.error());
        }
        return input.value();
    }

    template <typename T>
    bool has_value(std::string name)
    {
        BT::Optional<T> input = bt_node_handle_->getInput<T>(name);
        return input.has_value();
    }

    template <typename T>
    void set_value(std::string name, T value)
    {
        bt_node_handle_->setOutput<T>(name, value);
    }

private:
    BT::TreeNode *bt_node_handle_;
};
