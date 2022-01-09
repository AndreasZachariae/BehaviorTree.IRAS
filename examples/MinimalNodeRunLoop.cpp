/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : Examples
 * Purpose : Example of a minimal ROS2-Node class with 
 *           spin_some() method instead of blocking spin()
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#include "rclcpp/rclcpp.hpp"

class MinimalNodeRunLoop
{
private:
    std::shared_ptr<rclcpp::Node> node_handle_;

public:
    MinimalNodeRunLoop();
    ~MinimalNodeRunLoop();
    void run();
};

MinimalNodeRunLoop::MinimalNodeRunLoop()
{
    node_handle_ = rclcpp::Node::make_shared("MinimalNodeRunLoop");
}

MinimalNodeRunLoop::~MinimalNodeRunLoop()
{
    rclcpp::shutdown();
}

void MinimalNodeRunLoop::run()
{
    while (rclcpp::ok())
    {
        //publish etc...
        rclcpp::spin_some(node_handle_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    MinimalNodeRunLoop node;
    node.run();
    return 0;
}