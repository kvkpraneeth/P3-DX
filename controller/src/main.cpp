#include "controller/controller.h"
#include <cstdio>
#include <memory>

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<differential_drive> diffDriveNode = std::make_shared<differential_drive>();

    exe.add_node(diffDriveNode->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
