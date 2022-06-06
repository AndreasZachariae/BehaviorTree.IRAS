/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : behaviortree_ros
 * Purpose : Template for using the ROS2 action in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_ros/components/RosNode.h>

template <typename ActionT>
class RosAction : public RosNode
{
public:
    RosAction(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
    {
        progress_.steps = 3;
    }

protected:
    using GoalT = typename ActionT::Goal;
    using FeedbackT = typename ActionT::Feedback;
    using ResultT = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

    virtual void on_send(GoalT &goal) = 0;
    virtual void on_feedback(const std::shared_ptr<const FeedbackT>) {}
    virtual void on_result(const ResultT &result, const GoalT &goal) = 0;

private:
    using OptionsT = typename rclcpp_action::Client<ActionT>::SendGoalOptions;

    Progress progress_;

    std::shared_ptr<rclcpp_action::Client<ActionT>> client_;
    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>> future_;
    GoalT goal_;
    ResultT result_;

    BT::NodeStatus on_start() override
    {
        log("Connecting to action server: " + ros_name());

        client_ = rclcpp_action::create_client<ActionT>(
            get_node_handle()->get_node_base_interface(),
            get_node_handle()->get_node_graph_interface(),
            get_node_handle()->get_node_logging_interface(),
            get_node_handle()->get_node_waitables_interface(),
            ros_name());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus on_running() override
    {
        switch (progress_.current_step)
        {
        case Progress::FAIL_STEP:
            progress_.set_step(0, "Reset");
            return BT::NodeStatus::FAILURE;
        case 0:
            if (client_->action_server_is_ready())
            {
                send_();
            }
            break;
        case 3:
            progress_.set_step(0, "Reset");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void on_halted() override
    {
        try
        {
            client_->async_cancel_goal(future_.get());
        }
        catch (const std::future_error &e)
        {
            log("Canceled before goal was sent.");
        }
    }

    void send_()
    {
        OptionsT options = OptionsT();

        options.goal_response_callback = [&](std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>>)
        {
            if (!future_.get())
            {
                progress_.set_fail("Goal was rejected!");
            }
            else
            {
                progress_.next_step("Goal accepted.");
            }
        };

        options.feedback_callback = [&](std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>, const std::shared_ptr<const FeedbackT> feedback)
        {
            on_feedback(feedback);
        };

        options.result_callback = [&](const ResultT &result)
        {
            result_ = result;

            if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                progress_.set_fail("Goal canceled!");
            }
            else if (result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                progress_.set_fail("Goal failed! ResultCode: UNKNOWN");
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                progress_.set_fail("Goal failed! ResultCode: ABORTED");
            }
            else
            {
                on_result(result_, goal_);

                progress_.next_step("Received result.");
            }
        };

        goal_ = GoalT();

        on_send(goal_);

        future_ = client_->async_send_goal(goal_, options);

        progress_.next_step("Goal sent.");
    }
};