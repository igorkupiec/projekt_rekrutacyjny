#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class joyToTwistSub: public rclcpp::Node{
    public:
        joyToTwistSub();
    
    private:
        void callback_input(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void callback_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void callbackfunction();
        
    
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_input_, subscription_velocity_;
    geometry_msgs::msg::TwistStamped last_input_msg_, last_velocity_msg_;
};