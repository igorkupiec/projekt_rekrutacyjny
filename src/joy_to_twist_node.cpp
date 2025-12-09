#include "joy_to_twist/joy_to_twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

joy_to_twist_node::joy_to_twist_node()
    :Node("joy_to_twist_node"){
    // getting last callback time 
    last_callback_time = this->now();
    // create subscription to joy topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&joy_to_twist_node::callbackfunction, this, _1));
    // create publisher for "input_pada" topic
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("input_pada", 10);
    // create publisher for "robot_velocity" topic
    publisher_velocity_ = this -> create_publisher<geometry_msgs::msg::TwistStamped>("robot_velocity", 10);
    // create timer for checking if joy node is working
    timer_ = this->create_wall_timer(500ms, std::bind(&joy_to_twist_node::check_joy, this));
};


// check_joy -> if joy node doesnt send any data for more than 0.1s function outputs info
void joy_to_twist_node::check_joy(){
    auto time_diff = this->now() - last_callback_time;
      if(time_diff.seconds() > 0.1){
        RCLCPP_WARN(this->get_logger(), "joy_node nie wysyla inputow");
      }
}

void joy_to_twist_node::callbackfunction(const sensor_msgs::msg::Joy::SharedPtr msg){
    //info -> joy node sends data
    RCLCPP_INFO_ONCE(this->get_logger(), "Przesylam dane z joy_node");
    //last function callback time
    last_callback_time = this->now();

    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    auto twist_velocity = geometry_msgs::msg::TwistStamped(); 

    twist_stamped_msg.header.stamp = this->get_clock()->now();

    twist_velocity.header.stamp = this->get_clock()->now();

    if(msg->axes.size() >= 4){
      //left stick
      twist_stamped_msg.twist.angular.x = msg->axes[0]*-1;
      twist_stamped_msg.twist.angular.y = msg->axes[1];

      //robot velocity
      double velocity = 1 * sqrt(abs(msg->axes[1]));
      if(msg->axes[1] > 0){
        twist_velocity.twist.linear.x = velocity;
      }
      else{
        twist_velocity.twist.linear.x = velocity * -1;
      }
      //robot rotation velocity
      double rvelocity = 1 * sqrt(abs(msg->axes[2]));
      if(msg->axes[2] < 0){
        twist_velocity.twist.angular.z = rvelocity;
      }
      else{
        twist_velocity.twist.angular.z = rvelocity * -1;
      }
      
      //right stick
      twist_stamped_msg.twist.linear.x = msg->axes[2]*-1;
      twist_stamped_msg.twist.linear.y = msg->axes[3];        
    }
    //publishes TwistStamped msg to "input_pada" topic
    publisher_ -> publish(twist_stamped_msg);

    //publishes TwistStamped robot velocity info to "robot_velocity" topic
    publisher_velocity_ -> publish(twist_velocity);
}