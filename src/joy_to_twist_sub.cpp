#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;

class joy_to_twist_sub : public rclcpp::Node{
    public:
        joy_to_twist_sub()
        :Node("joy_to_twist_sub"){
            subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("input_pada", 10, std::bind(&joy_to_twist_sub::callbackfunction, this, _1));
        }
    private:
        void callbackfunction(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Drazek lewy x: '%.2f' y: '%.2f' || Drazek prawy x: '%.2f' y: '%.2f'", msg->twist.angular.x, msg->twist.angular.y, msg->twist.linear.x, msg->twist.linear.y);
            
        }
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joy_to_twist_sub>());
    rclcpp::shutdown();

    return 0;
}