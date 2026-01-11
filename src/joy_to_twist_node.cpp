#include "joy_to_twist/joy_to_twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

joyToTwistNode::joyToTwistNode()
    :Node("joy_to_twist_node"){
    // Declare parameters
    this->declare_parameter<std::string>("joy_topic", "joy");
    this->declare_parameter<std::string>("input_topic", "input_pada");
    this->declare_parameter<std::string>("vel_topic", "robot_velocity");

    // Get the parameters
    std::string joy_t = this->get_parameter("joy_topic").as_string();
    std::string input_t = this->get_parameter("input_topic").as_string();
    std::string vel_t = this->get_parameter("vel_topic").as_string();

    // getting last callback time 
    last_callback_time = this->now();
    // create subscription to joy topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_t, 10, std::bind(&joyToTwistNode::callbackfunction, this, _1));
    // create publisher for "input_pada" topic
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(input_t, 10);
    // create publisher for "robot_velocity" topic
    publisher_velocity_ = this -> create_publisher<geometry_msgs::msg::TwistStamped>(vel_t, 10);
    // create timer for checking if joy node is working
    timer_ = this->create_wall_timer(500ms, std::bind(&joyToTwistNode::check_joy, this));
};


// check_joy -> if joy node doesnt send any data for more than 0.1s function outputs info
void joyToTwistNode::check_joy(){
    auto time_diff = this->now() - last_callback_time;
      if(time_diff.seconds() > 0.1){
        RCLCPP_WARN(this->get_logger(), "joy_node nie wysyla inputow");
      }
}

void joyToTwistNode::callbackfunction(const sensor_msgs::msg::Joy::SharedPtr msg){
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
      //right stick
      twist_stamped_msg.twist.linear.x = msg->axes[2]*-1;
      twist_stamped_msg.twist.linear.y = msg->axes[3]; 

      //robot velocity
      double velocity = 1 * pow(abs(msg->axes[1]), 2);
      if(msg->axes[1] > 0){
        twist_velocity.twist.linear.x = velocity;
      }
      else{
        twist_velocity.twist.linear.x = velocity * -1;
      }
      //robot rotation velocity
      double rvelocity = 1 * pow(abs(msg->axes[2]), 2);
      if(msg->axes[2] < 0){
        twist_velocity.twist.angular.z = rvelocity;
      }
      else{
        twist_velocity.twist.angular.z = rvelocity * -1;
      }
    }
    //publishes TwistStamped msg to "input_pada" topic
    publisher_ -> publish(twist_stamped_msg);

    //publishes TwistStamped robot velocity info to "robot_velocity" topic
    publisher_velocity_ -> publish(twist_velocity);
}