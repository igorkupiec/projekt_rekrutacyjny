#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class joy_to_twist_node: public rclcpp::Node{
  public:
    joy_to_twist_node()
    :Node("joy_to_twist_node"){
      //ile przyciskow nasluchuje od joja
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",10, std::bind(&joy_to_twist_node::callbackfunction, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("input_pada", 10);
      
      //sprawdzanie co jakis czas czy joy_node dziala
      timer_ = this->create_wall_timer(500ms, std::bind(&joy_to_twist_node::check_joy, this));
    }
  
  private:
    void check_joy(){
      auto time_diff = this->now() - last_callback_time;
      if(time_diff.seconds() > 0.1){
        RCLCPP_WARN(this->get_logger(), "joy_node nie wysyla inputow");
      }
    }
    void callbackfunction(const sensor_msgs::msg::Joy::SharedPtr msg){
      RCLCPP_INFO_ONCE(this->get_logger(), "Przesylam dane z joy_node");
      //ostatni czas wywolania funkcji
      last_callback_time = this->now();

      auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
      
      twist_stamped_msg.header.stamp = this->get_clock()->now();

      if(msg->axes.size() >= 4){
        //lewa galka
        twist_stamped_msg.twist.angular.x = msg->axes[0]*-1;
        twist_stamped_msg.twist.angular.y = msg->axes[1];
        //prawa galka
        twist_stamped_msg.twist.linear.x = msg->axes[3]*-1;
        twist_stamped_msg.twist.linear.y = msg->axes[4];        
      }
      //publikuje temat twiststamped wiadomosci
      publisher_ -> publish(twist_stamped_msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Time last_callback_time = this->now();
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joy_to_twist_node>());
  rclcpp::shutdown();

  return 0;
}
