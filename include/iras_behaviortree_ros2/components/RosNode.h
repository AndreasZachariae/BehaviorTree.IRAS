/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Wrapper for a node in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <iras_behaviortree_ros2/default.h>

#include <cmath>

#include <behaviortree_cpp_v3/action_node.h>

#include <iras_behaviortree_ros2/tools/Progress.h>
#include <iras_behaviortree_ros2/tools/Converter.h>
#include <iras_behaviortree_ros2/tools/PortHandler.h>
#include <iras_behaviortree_ros2/components/RosInterface.h>

class RosNode : public RosInterface, public BT::StatefulActionNode
{
public:
    RosNode(const std::string &name, const BT::NodeConfiguration &config) : RosInterface(name), BT::StatefulActionNode(name, config) {}

protected:
    PortHandler ports = PortHandler(dynamic_cast<BT::TreeNode *>(this));
    Progress progress_;

    virtual BT::NodeStatus on_start() = 0;
    virtual BT::NodeStatus on_running() = 0;
    virtual void on_halted() {}

private:
    virtual BT::NodeStatus onStart() override
    {
        progress_.current_step = 0;

        check_required_nodes();

        return on_start();
    }

    virtual BT::NodeStatus onRunning() override { return on_running(); }

    virtual void onHalted() override { on_halted(); }
};
