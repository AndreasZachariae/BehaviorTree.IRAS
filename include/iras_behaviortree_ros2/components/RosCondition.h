/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Wrapper for Conditions in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <iras_behaviortree_ros2/default.h>

#include <behaviortree_cpp_v3/condition_node.h>

#include <iras_behaviortree_ros2/tools/Converter.h>
#include <iras_behaviortree_ros2/tools/PortHandler.h>
#include <iras_behaviortree_ros2/components/RosInterface.h>

class RosCondition : public RosInterface, public BT::ConditionNode
{
public:
    RosCondition(const std::string &name, const BT::NodeConfiguration &config) : RosInterface(name), BT::ConditionNode(name, config) {}

    virtual BT::NodeStatus on_check() = 0;

protected:
    PortHandler ports = PortHandler(dynamic_cast<BT::TreeNode *>(this));

private:
    BT::NodeStatus tick() override
    {
        check_required_nodes();

        return on_check();
    }
};