/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : behaviortree_ros
 * Purpose : Wrapper for Conditions in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <behaviortree_cpp_v3/condition_node.h>

#include <behaviortree_ros/components/RosInterface.h>

class RosCondition : public RosInterface, public BT::ConditionNode
{
public:
    RosCondition(const std::string &name, const BT::NodeConfiguration &config) : RosInterface(name), BT::ConditionNode(name, config) {}

    virtual BT::NodeStatus on_check() = 0;

private:
    BT::NodeStatus tick() override
    {
        check_required_nodes();

        return on_check();
    }
};