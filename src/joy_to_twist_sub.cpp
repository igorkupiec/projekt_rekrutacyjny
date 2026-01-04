#include "joy_to_twist/joy_to_twist_sub.hpp"

using std::placeholders::_1;

joyToTwistSub::joyToTwistSub()
    :Node("joy_to_twist_sub"){
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "input_pada");
    this->declare_parameter<std::string>("vel_topic", "robot_velocity");

    // Get topic names
    std::string input_t = this->get_parameter("input_topic").as_string();
    std::string vel_t = this->get_parameter("vel_topic").as_string();

    //create subsciption to "input_pada" topic
    subscription_input_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(input_t, 10, std::bind(&joyToTwistSub::callback_input, this, _1));
    //subscribe to "robot_velocity" topic
    subscription_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(vel_t, 10, std::bind(&joyToTwistSub::callback_velocity, this, _1));
};  

void joyToTwistSub::callback_input(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    //Get the stick data
    last_input_msg_ = *msg;
    callbackfunction();
}

void joyToTwistSub::callback_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    //Get the velocity and rotation
    last_velocity_msg_ = *msg;
    callbackfunction();
}

void joyToTwistSub::callbackfunction() 
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