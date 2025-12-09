#include "joy_to_twist/joy_to_twist_sub.hpp"

using std::placeholders::_1;

joy_to_twist_sub::joy_to_twist_sub()
    :Node("joy_to_twist_sub"){
    //create subsciption to "input_pada" topic
    subscription_input_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("input_pada", 10, std::bind(&joy_to_twist_sub::callback_input, this, _1));
    //subscribe to "robot_velocity" topic
    subscription_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("robot_velocity", 10, std::bind(&joy_to_twist_sub::callback_velocity, this, _1));
};  

void joy_to_twist_sub::callback_input(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    //Get the stick data
    last_input_msg_ = *msg;
    callbackfunction();
}

void joy_to_twist_sub::callback_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    //Get the velocity and rotation
    last_velocity_msg_ = *msg;
    callbackfunction();
}

void joy_to_twist_sub::callbackfunction() 
{
    // print the pad input and velocity data
    RCLCPP_INFO(this->get_logger(), 
        "Drazek lewy x: '%.2f' y: '%.2f' || Drazek prawy x: '%.2f' y: '%.2f' || Predkosc: '%.2f' || Obrot: '%.2f'", 
        last_input_msg_.twist.angular.x, 
        last_input_msg_.twist.angular.y, 
        last_input_msg_.twist.linear.x, 
        last_input_msg_.twist.linear.y, 
        last_velocity_msg_.twist.linear.x, 
        last_velocity_msg_.twist.angular.z
    );
}