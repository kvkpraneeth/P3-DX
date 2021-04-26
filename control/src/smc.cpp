#include "control/smc.h"
#include <functional>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <math.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
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
    
    this->stateSub = this->create_subscription<geometry_msgs::msg::TransformStamped>("/pioneerState", 100, std::bind(&slidingModeControl::__stateCB, this, std::placeholders::_1));
    this->timer = this->create_wall_timer(this->frequency, std::bind(&slidingModeControl::__timerCB, this));      
    this->speedTimer = this->create_wall_timer(this->frequency, std::bind(&slidingModeControl::__speedTimerCB, this));      
   
    this->k1 = 5.0;
    this->k2 = 2.0;

    this->speedPub = this->create_publisher<geometry_msgs::msg::Twist>(this->commandSpeedTopic, 100);   
    this->speedEstimatePub = this->create_publisher<geometry_msgs::msg::Twist>(this->speedEstimateTopic, 100);
    this->desiredSpeedPub = this->create_publisher<geometry_msgs::msg::Twist>(this->desiredSpeedTopic, 100);   

    this->currentTime = now();    
    this->lastTime = now();

    this->publishSpeed.angular.x = 0.0;
    this->publishSpeed.angular.y = 0.0;
    this->publishSpeed.angular.z = 0.0;
    this->publishSpeed.linear.x = 0.0;
    this->publishSpeed.linear.y = 0.0;
    this->publishSpeed.linear.z = 0.0;

    this->desiredSpeed.angular.x = 0.0;
    this->desiredSpeed.angular.y = 0.0;
    this->desiredSpeed.angular.z = 1.0;
    this->desiredSpeed.linear.x = 2.0;
    this->desiredSpeed.linear.y = 0.0;
    this->desiredSpeed.linear.z = 0.0;

}

void slidingModeControl::__stateCB(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    this->previousState = this->currentState;
    this->currentState.header = msg->header;
    this->currentState.transform = msg->transform;
}

void slidingModeControl::__timerCB()
{

    currentTime = now();

    rclcpp::Duration dt = currentTime - lastTime;

    double dthd = desiredSpeed.angular.z * dt.seconds();
    double dxd = desiredSpeed.linear.x * cos(thd) * dt.seconds();
    double dyd = desiredSpeed.linear.x * sin(thd) * dt.seconds();    

    xd = currentState.transform.translation.x + dxd;
    yd = currentState.transform.translation.y + dyd;
    thd = quatToYaw(currentState) + dthd;

    double xe = (currentState.transform.translation.x - xd)*(cos(quatToYaw(currentState)));
    double ye = (currentState.transform.translation.y - yd)*(cos(quatToYaw(currentState)));
    double the = quatToYaw(currentState) - thd;

    RCLCPP_INFO(this->get_logger(), "lmao: '%f'", quatToYaw(currentState));

    publishSpeed.linear.x = (-publishSpeed.angular.z)*(xe + ye) + (desiredSpeed.linear.x * (cos(the) + sin(the))) - (this->k1) *  (tanh(xe - ye));
    publishSpeed.angular.z = desiredSpeed.angular.z - (this->k2 * tanh(the - xe + ye));

    speedPub->publish(publishSpeed);
    desiredSpeedPub->publish(desiredSpeed);

    lastTime = currentTime;

}

void slidingModeControl::__speedTimerCB()
{
    double dx=this->currentState.transform.translation.x-this->previousState.transform.translation.x;
    double dth=quatToYaw(currentState)-quatToYaw(previousState);

    currentSpeed.linear.x = dx/cos(quatToYaw(currentState));
    currentSpeed.angular.z = dth;

    speedEstimatePub->publish(currentSpeed);
}
