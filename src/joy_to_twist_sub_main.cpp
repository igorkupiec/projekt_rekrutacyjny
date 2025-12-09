#include "rclcpp/rclcpp.hpp"
#include <joy_to_twist/joy_to_twist_sub.hpp>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joy_to_twist_sub>());
    rclcpp::shutdown();
    return 0;
}