#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <iomanip>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <math.h>

class joy_to_twist_tui: public rclcpp::Node{
    public:
        joy_to_twist_tui();
    
    private:
        void callbackfunction(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void drazek_tui(double xl, double yl, double xr, double yr, int h, int w);
    
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    struct winsize w;
};