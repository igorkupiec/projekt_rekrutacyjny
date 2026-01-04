#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class joyToTwistNode: public rclcpp::Node{
    public: 
        joyToTwistNode();

    private:
        void check_joy();
        void callbackfunction(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_, publisher_velocity_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_callback_time;
};