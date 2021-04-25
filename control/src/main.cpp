#include "control/smc.h"
#include <cstdio>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/clock.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<slidingModeControl> slidingModeControlNode = std::make_shared<slidingModeControl>();

    exe.add_node(slidingModeControlNode->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
