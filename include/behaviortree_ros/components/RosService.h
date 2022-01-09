/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : behaviortree_ros
 * Purpose : Template for using the ROS2 service in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <memory>

// #include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_ros/components/RosNode.h>

template <typename ServiceT>
class RosService : public RosNode
{
public:
    RosService(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
    {
        progress_.steps = 2;
    }

protected:
    using RequestT = typename ServiceT::Request;
    using ResponseT = typename ServiceT::Response;

    virtual void on_send(std::shared_ptr<RequestT> request) = 0;
    virtual bool on_result(std::shared_future<std::shared_ptr<ResponseT>> response, std::shared_ptr<RequestT> request) = 0;

private:
    Progress progress_;

    std::shared_ptr<rclcpp::Client<ServiceT>> client_;
    std::shared_ptr<RequestT> request_;
    std::shared_future<std::shared_ptr<ResponseT>> response_;
    bool success_ = true;

    BT::NodeStatus on_start() override
    {
        log("Connecting to service: " + ros_name());

        client_ = RosNode::get_node_handle()->create_client<ServiceT>(ros_name());

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
            if (client_->service_is_ready())
            {
                send_();
            }
            break;
        case 2:
            progress_.set_step(0, "Reset");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void send_()
    {
        request_ = std::make_shared<RequestT>();

        on_send(request_);

        response_ = client_->async_send_request(request_, [this](std::shared_future<std::shared_ptr<ResponseT>>)
                                                {
                                                    success_ = on_result(response_, request_);

                                                    progress_.next_step("Response received.");

                                                    if (!success_)
                                                    {
                                                        progress_.set_fail("Service");
                                                    }
                                                });

        progress_.next_step("Request sent.");
    }
};