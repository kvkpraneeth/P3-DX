#include "control/control.h"

using namespace std;

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exe;

	shared_ptr<TrajectoryTracking> tt = make_shared<TrajectoryTracking>();

	exe.add_node(tt->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}
