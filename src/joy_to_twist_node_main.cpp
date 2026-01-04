#include "rclcpp/rclcpp.hpp"
#include <joy_to_twist/joy_to_twist.hpp>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joyToTwistNode>());
    rclcpp::shutdown();
    return 0;
}