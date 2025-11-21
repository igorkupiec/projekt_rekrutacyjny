#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <iomanip>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
using std::placeholders::_1;

class joy_to_twist_tui : public rclcpp::Node {
public:
    joy_to_twist_tui()
    : Node("joy_to_twist_tui") {
        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "input_pada", 
            10, 
            std::bind(&joy_to_twist_tui::callbackfunction, this, _1));
    }

private:
    void callbackfunction(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
        double lx = msg->twist.angular.x, ly = msg->twist.angular.y, rx = msg->twist.linear.x, ry = msg->twist.linear.y;
        drazek_tui(lx, ly, rx, ry, w.ws_row, w.ws_col);
    }
    void drazek_tui(double xl, double yl, double xr, double yr, int h, int w){
        std::string klatka;
        klatka += "\033[H";
        int r = h/(5/2), rd=h/3, l_cx=w/4, r_cx = (w*3)/4, cy=h/2;
        int x2 = l_cx + (int)(xl * r/2 * 2.0); 
        int y2 = cy - (int)(yl * r/2);
        int x3 = r_cx + (int)(xr * r/2 * 2.0); 
        int y3 = cy - (int)(yr * r/2);
        

        for (int iy = 0; iy < h; iy++) {
            for (int jx = 0; jx < w; jx++) {
                //glowny lewy okrag
                double dy = (iy - cy) * 2.0;
                double l_dx = (jx - l_cx);
                double l_d = l_dx*l_dx + dy*dy;
                //glowny prawy okrag
                double r_dx = (jx - r_cx);
                double r_d = r_dx*r_dx + dy*dy;
                //drazek lewy
                double l_dyd = (iy - y2) * 2.0; 
                double l_dxd = (jx - x2);
                double l_dd = l_dxd*l_dxd + l_dyd*l_dyd;
                //drazek prawy
                double r_dyd = (iy - y3) * 2.0; 
                double r_dxd = (jx - x3);
                double r_dd = r_dxd*r_dxd + r_dyd*r_dyd;
                bool polowa = (jx < w/2);
                
                if(polowa){
                    if (l_dd <= rd) {
                        klatka += "@"; 
                    } 
                    else if (l_d <= r*r + 20.0 && l_d >= r*r - 80.0) { 
                        klatka += "*"; 
                    } 
                    else if (iy == cy && jx == l_cx) {
                        klatka += "+"; 
                    }
                    else {
                        klatka += " "; 
                    }
                }
                else{
                    if (r_dd <= rd) {
                        klatka += "@"; 
                    } 
                    else if (r_d <= r*r + 20.0 && r_d >= r*r - 80.0) { 
                        klatka += "*"; 
                    } 
                    else if (iy == cy && jx == r_cx) {
                        klatka += "+"; 
                    }
                    else {
                        klatka += " "; 
                    }
                }
                
            }
            klatka += "\n";
        }std::cout<<klatka;
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    struct winsize w;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joy_to_twist_tui>());
    rclcpp::shutdown();
    return 0;
}