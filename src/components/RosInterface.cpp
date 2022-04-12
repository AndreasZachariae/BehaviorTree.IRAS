#include <behaviortree_ros/components/RosInterface.h>

#include <cpp_core/tools/CommandLine.h>

rclcpp::Node::SharedPtr RosInterface::node_handle_ = nullptr;

std::string RosInterface::node_name = "TaskPlanner";

uint RosInterface::instance_count_ = 0;

void RosInterface::init(std::string node_name, int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RosInterface::node_name = node_name;
}

rclcpp::Node::SharedPtr RosInterface::get_node_handle()
{
    if (RosInterface::node_handle_ == nullptr)
    {
        RosInterface::node_handle_ = std::make_shared<rclcpp::Node>(RosInterface::node_name);
    }

    return RosInterface::node_handle_;
}

RosInterface::RosInterface(const std::string &name) : Component(name)
{
    if (instance_count_++ == 0 && !rclcpp::ok())
    {
        init();
    }
}

RosInterface::~RosInterface()
{
    if (--instance_count_ == 0)
    {
        shutdown();
    }
}

void RosInterface::check_required_nodes()
{
    if (required_nodes.size() == 0)
    {
        return;
    }

    while (true)
    {
        // Get existing nodes
        std::string response = CommandLine::execute("ros2 node list");

        std::vector<std::string> existing_nodes = CommandLine::split_string(response, "\n");

        // Check missing nodes
        std::vector<std::string> missing_nodes;

        for (std::string required_node : required_nodes)
        {
            bool exists = false;

            for (std::string existing_node : existing_nodes)
            {
                if (required_node == existing_node)
                {
                    exists = true;
                    break;
                }
            }

            if (!exists)
            {
                missing_nodes.push_back(required_node);
            }
        }

        if (missing_nodes.empty())
        {
            break;
        }

        // Log missing nodes
        std::string text = "Missing nodes:";

        for (std::string missing_node : missing_nodes)
        {
            text += "\n" + missing_node;
        }

        log(text);
    }
}
