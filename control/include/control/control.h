#ifndef CONTROL_H
#define CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <vector>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "p3dxmsgs/srv/state.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/float32.h"

#define pi 3.14159

typedef p3dxmsgs::srv::State State;
typedef geometry_msgs::msg::TransformStamped TransformStamped;
typedef std_msgs::msg::Float32 Float32;

class TrajectoryTracking : public rclcpp::Node
{

	public:
		// Constructor
		TrajectoryTracking();
		
		// Desired State Service Callback	
		void getDesiredState(const State::Request::SharedPtr request, 
							State::Response::SharedPtr response);

		// Current State Subscriber Callback
		void getCurrentState(const TransformStamped::SharedPtr msg);

	private:	

		// Desired State
		State::Request desiredState;

		const std::string desiredStateServiceTopic = "desiredState";
	
		rclcpp::Service<State>::SharedPtr desiredStateService;

		// Current State
		rclcpp::Subscription<TransformStamped>::SharedPtr currentStateSub;
		const std::string currentStateTopic = "pioneerState";

		// Worker Functions
			//ctdsf -> Convert To Desired State Format
		State::Request ctdsf(const TransformStamped::SharedPtr State);

		// Wheel Velocity Publications
		rclcpp::Publisher<Float32>::SharedPtr leftMotorPub;			
		rclcpp::Publisher<Float32>::SharedPtr rightMotorPub;			

		const std::string leftMotorTopic = "leftMotorSpeed";
		const std::string rightMotorTopic = "rightMotorSpeed";

		// P3-DX Parameters
		const double wheelDiameter=0.195;
		const double wheelSeparation=0.331;

};



#endif
