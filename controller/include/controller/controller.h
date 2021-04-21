#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <string>
#include "rclcpp/timer.hpp"

class differential_drive : public rclcpp::Node
{

    //Constructor to start the drive node.
    public: differential_drive();

    //Crucial Topics.
    public: std::string leftWheelTopic="leftMotorSpeed", rightWheelTopic="rightMotorSpeed", speedCommandTopic="cmd_vel";

    //Robot Specifications.
    public: float wheelDiameter=0.195, wheelSeparation=0.331;

    //Internal Variables from Callbacks and to Callbacks.
    private: std_msgs::msg::Float32 leftWheelSpeed, rightWheelSpeed;

    //Callbacks.
    public: void __speedCommandCB(const geometry_msgs::msg::Twist::SharedPtr msg);
            void __timerCB();

    //Subscriptions.
    private: rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speedSub;

    //Publications.
    private: rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftWheelPub;
             rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightWheelPub;
             rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPub;

    //Timer based Loops.
    private: std::shared_ptr<rclcpp::TimerBase> timerLoop;
             std::chrono::milliseconds frequency{5};

    //Dead Reckoning.
    public: void __odometry(float vx, float vy, float vth);

};

#endif
