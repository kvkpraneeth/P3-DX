#include "control/smc.h"
#include <functional>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <math.h>
#include <rclcpp/duration.hpp>
#include "cmath"
#include "tf2/LinearMath/Quaternion.h"

double quatToYaw(geometry_msgs::msg::TransformStamped msg)
{

    double w,x,y,z;

    w = msg.transform.rotation.w;
    x = msg.transform.rotation.x;
    y = msg.transform.rotation.y;
    z = msg.transform.rotation.z;

    double yaw;

    yaw = std::atan2(2*(w*z + x*y), (1-(2*(y*y + z*z))));

    return yaw;

}

slidingModeControl::slidingModeControl() : Node("SlidingModeControl")
{
    
    this->stateSub = this->create_subscription<geometry_msgs::msg::TransformStamped>("/tf", 100, std::bind(&slidingModeControl::__stateCB, this, std::placeholders::_1));
    this->timer = this->create_wall_timer(this->frequency, std::bind(&slidingModeControl::__timerCB, this));      
    
    this->k1 = 5.0;
    this->k2 = 2.0;

    this->speedPub = this->create_publisher<geometry_msgs::msg::Twist>(this->commandSpeedTopic, 100);   

    this->currentTime = now();    
    this->lastTime = now();

    this->currentSpeed.angular.x = 0.0;
    this->currentSpeed.angular.y = 0.0;
    this->currentSpeed.angular.z = 0.0;
    this->currentSpeed.linear.x = 0.0;
    this->currentSpeed.linear.y = 0.0;
    this->currentSpeed.linear.z = 0.0;

    this->desiredSpeed.angular.x = 0.0;
    this->desiredSpeed.angular.y = 0.0;
    this->desiredSpeed.angular.z = 0.0;
    this->desiredSpeed.linear.x = 2.0;
    this->desiredSpeed.linear.y = 0.0;
    this->desiredSpeed.linear.z = 0.0;

}

void slidingModeControl::__stateCB(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    this->currentState.header = msg->header;
    this->currentState.transform = msg->transform;
}

void slidingModeControl::__timerCB()
{

    currentTime = now();

    rclcpp::Duration dt = currentTime - lastTime;

    double dthd = desiredSpeed.angular.z * dt.seconds();
    double dxd = desiredSpeed.linear.x * cos(dthd) * dt.seconds();
    double dyd = desiredSpeed.linear.x * sin(dthd) * dt.seconds();    

    double xd = currentState.transform.translation.x + dxd;
    double yd = currentState.transform.translation.y + dyd;
    double thd = quatToYaw(currentState) + dthd;

    double xe = (currentState.transform.translation.x - xd)*(cos(quatToYaw(currentState))) + (currentState.transform.translation.y - yd)*(sin(quatToYaw(currentState)));
    double ye = (currentState.transform.translation.y - yd)*(cos(quatToYaw(currentState))) - (currentState.transform.translation.x)*(sin(quatToYaw(currentState)));
    double the = quatToYaw(currentState) - thd;

    currentSpeed.linear.x = (-currentSpeed.angular.z)*(xe + ye) + (desiredSpeed.linear.x * (cos(the) + sin(the))) - (this->k1) *  (tanh(xe - ye));
    currentSpeed.angular.z = desiredSpeed.angular.z - (this->k2 * tanh(the - xe + ye));
    
    speedPub->publish(currentSpeed);

    lastTime = currentTime;

}

