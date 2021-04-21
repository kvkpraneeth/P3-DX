#include "controller/controller.h"

differential_drive::differential_drive() : Node("DifferentialDrive")
{
    
    this->speedSub = this->create_subscription<geometry_msgs::msg::Twist>(this->speedCommandTopic, 100, std::bind(&differential_drive::__speedCommandCB, this, std::placeholders::_1));
    this->leftWheelPub = this->create_publisher<std_msgs::msg::Float32>(this->leftWheelTopic, 100);
    this->rightWheelPub = this->create_publisher<std_msgs::msg::Float32>(this->rightWheelTopic, 100);
    this->timerLoop = this->create_wall_timer(this->frequency, std::bind(&differential_drive::__timerCB, this));

}

void differential_drive::__speedCommandCB(const geometry_msgs::msg::Twist::SharedPtr msg)
{

    this->leftWheelSpeed.data = (2 * msg->linear.x - this->wheelSeparation * msg->angular.z)/(this->wheelDiameter);
    this->rightWheelSpeed.data = (2 * msg->linear.x + this->wheelSeparation * msg->angular.z)/(this->wheelDiameter);

}

void differential_drive::__timerCB()
{

    this->leftWheelPub->publish(this->leftWheelSpeed);
    this->rightWheelPub->publish(this->rightWheelSpeed);

}

