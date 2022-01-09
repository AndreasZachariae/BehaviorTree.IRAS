/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : behaviortree_ros
 * Purpose : Wrapper for a node in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <cmath>

#include <behaviortree_cpp_v3/action_node.h>

#include <behaviortree_ros/tools/Progress.h>
#include <behaviortree_ros/components/RosInterface.h>

class RosNode : public RosInterface, public BT::StatefulActionNode
{
public:
    RosNode(const std::string &name, const BT::NodeConfiguration &config) : RosInterface(name), BT::StatefulActionNode(name, config) {}

protected:
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