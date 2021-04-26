#ifndef SMC_H
#define SMC_H

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/detail/tf_message__struct.hpp>

class slidingModeControl : public rclcpp::Node
{
    //Constructor to start the Controller
    public: slidingModeControl();

    //Crucial Topics.
    public: std::string commandSpeedTopic="cmd_vel";
            std::string speedEstimateTopic="vel_ach";
            std::string desiredSpeedTopic="des_vel";

    //Crucial Constants.
    public: float k1, k2;
    private: std::chrono::milliseconds frequency{5};

    public: std::string robotName="P3-DX";
            std::string worldName="world";

    //Robot Variables.
    public: geometry_msgs::msg::TransformStamped currentState;
            geometry_msgs::msg::Twist desiredSpeed;
            geometry_msgs::msg::Twist currentSpeed;
            geometry_msgs::msg::Twist publishSpeed;
            geometry_msgs::msg::TransformStamped previousState;
            double xd,yd,thd;

    //Subscriptions.
    private: rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr stateSub;

    //CallBacks.
    public: void __stateCB(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
            void __timerCB();
            void __speedTimerCB();

    //Timers.
    private: std::shared_ptr<rclcpp::TimerBase> timer;
             std::shared_ptr<rclcpp::TimerBase> speedTimer;

    //Publications.
    private: rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speedPub;
             rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speedEstimatePub;
             rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desiredSpeedPub;

    //Time Values
    public: rclcpp::Time currentTime, lastTime;

};

#endif
