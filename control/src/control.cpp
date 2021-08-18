#include "control/control.h"

using namespace std;

TrajectoryTracking::TrajectoryTracking() 
			: Node("Controller") 
{
	// TODO: Find a good method to make this code look good.
	desiredStateService = create_service<State>(desiredStateServiceTopic, 
							bind(&TrajectoryTracking::getDesiredState, this, 
										placeholders::_1, placeholders::_2));
	
	currentStateSub = create_subscription<TransformStamped>(currentStateTopic,
						100,bind(&TrajectoryTracking::getCurrentState, this,
									placeholders::_1));

	leftMotorPub = create_publisher<Float32>(leftMotorTopic, 100);
	rightMotorPub = create_publisher<Float32>(rightMotorTopic, 100);

	desiredState.x = 0;
	desiredState.y = 0;
	desiredState.yaw = 0;

}

void
TrajectoryTracking::getDesiredState(const State::Request::SharedPtr request,
									State::Response::SharedPtr response)
{
	(void)response;
	desiredState.x = request.get()->x;
	desiredState.y = request.get()->y;
	desiredState.yaw = request.get()->yaw;
}

State::Request
TrajectoryTracking::ctdsf(const TransformStamped::SharedPtr State)
{
	State::Request temp;

	double x = State.get()->transform.rotation.x;
	double y = State.get()->transform.rotation.y;
	double z = State.get()->transform.rotation.z;
	double w = State.get()->transform.rotation.w;

	temp.yaw = atan2(2*(w*z + x*y), (1-(2*(y*y + z*z))));

	temp.x = State.get()->transform.translation.x;
	temp.y = State.get()->transform.translation.y;	

	return temp;
}

void
TrajectoryTracking::getCurrentState(const TransformStamped::SharedPtr msg)
{
	State::Request currentState = ctdsf(msg);

	Eigen::MatrixXd errorState = Eigen::MatrixXd(3,1);
	
	// Error State Calculation
	errorState(0,0) = desiredState.x - currentState.x;	
	errorState(1,0) = desiredState.y - currentState.y;	
	errorState(2,0) = desiredState.yaw - currentState.yaw;	

	cout << errorState << endl;

	Eigen::MatrixXd controlInputs = Eigen::MatrixXd(2,1);

	cout << controlInputs << endl;

	Float32 temp;
	
	temp.data = ((2*controlInputs(0,0)-wheelSeparation*controlInputs(1,0))
						/(wheelDiameter))*(pi/180);
	leftMotorPub.get()->publish(temp);	
	
	temp.data = ((2*controlInputs(0,0)+wheelSeparation*controlInputs(1,0))
						/(wheelDiameter))*(pi/180);
	rightMotorPub.get()->publish(temp);
}
