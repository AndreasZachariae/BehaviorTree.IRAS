#include "rclcpp/rclcpp.hpp"

class MinimalNodeInheritance : public rclcpp::Node
{
private:
public:
    MinimalNodeInheritance();
    ~MinimalNodeInheritance();
};

MinimalNodeInheritance::MinimalNodeInheritance() : Node("MinimalNodeInheritance")
{
}

MinimalNodeInheritance::~MinimalNodeInheritance()
{
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalNodeInheritance>());
    rclcpp::shutdown();
    return 0;
}