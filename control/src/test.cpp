#include "control/control.h"

using namespace std;

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exe;

	std::vector<double> Q,R;

	Q.resize(9);
	R.resize(4);

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			Q[3*i + j] = 1*(i==j);
		}
	}	
	
	for(int i=0; i<2; i++)
	{
		for(int j=0; j<2; j++)
		{
			R[2*i + j] = 1*(i==j);
		}
	}	

	shared_ptr<TrajectoryTracking> tt = make_shared<TrajectoryTracking>(Q, R);

	exe.add_node(tt->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}
