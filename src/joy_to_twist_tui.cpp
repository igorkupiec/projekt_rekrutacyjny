#include "joy_to_twist/joy_to_twist_tui.hpp"
#include <unistd.h>
#include <iostream>
#include <math.h>

using std::placeholders::_1;

joy_to_twist_tui::joy_to_twist_tui()
    :Node("joy_to_twist_tui"){
    //subscribe to "input_pada" topic
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "input_pada", 10, std::bind(&joy_to_twist_tui::callbackfunction, this, _1));
}

void joy_to_twist_tui::callbackfunction(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    //get window size of terminal
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    double lx = msg->twist.angular.x;
    double ly = msg->twist.angular.y; 
    double rx = msg->twist.linear.x; 
    double ry = msg->twist.linear.y;
    
    drazek_tui(lx, ly, rx, ry, w.ws_row, w.ws_col);
}

void joy_to_twist_tui::drazek_tui(double xl, double yl, double xr, double yr, int h, int w){
    //create frame
    std::string klatka;
    klatka += "\033[H";

    //radius of main circle
    int r = (int)(h / 2.5);  
    if (r == 0) r = 1; 

    //radius of inner circle
    int rd_linear = r / 5; 
    double rd_sq = (double)rd_linear * rd_linear; 
    //limits of inner circle
    double r_sq_max = (double)r * r + 20.0;
    double r_sq_min = (double)r * r - 80.0;

    //center position of left and right main circle
    int l_cx = w / 4;
    int r_cx = (w * 3) / 4;
    int cy = h / 2;

    //left inner circle position
    int x2 = l_cx + (int)(xl * r/2 * 2.0); 
    int y2 = cy - (int)(yl * r/2);
    //right inner circle position
    int x3 = r_cx + (int)(xr * r/2 * 2.0); 
    int y3 = cy - (int)(yr * r/2);

    for (int iy = 0; iy < h; iy++) {
        for (int jx = 0; jx < w; jx++) {
            double dy = (iy - cy) * 2.0;
            
            // Left main circle
            double l_dx = (jx - l_cx);
            double l_d_sq = l_dx*l_dx + dy*dy; 
            // Prawy glowny okrag
            double r_dx = (jx - r_cx);
            double r_d_sq = r_dx*r_dx + dy*dy;
            // Lewa kropka 
            double l_dyd = (iy - y2) * 2.0; 
            double l_dxd = (jx - x2);
            double l_dd_sq = l_dxd*l_dxd + l_dyd*l_dyd;
            // Prawa kropka 
            double r_dyd = (iy - y3) * 2.0; 
            double r_dxd = (jx - x3);
            double r_dd_sq = r_dxd*r_dxd + r_dyd*r_dyd;
            //which half of the screen
            bool polowa = (jx < w/2);
            
            if(polowa){
                if (l_dd_sq <= rd_sq) {
                    klatka += "@"; 
                } 
                else if (l_d_sq <= r_sq_max && l_d_sq >= r_sq_min) { 
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
                if (r_dd_sq <= rd_sq) {
                    klatka += "@"; 
                } 
                else if (r_d_sq <= r_sq_max && r_d_sq >= r_sq_min) { 
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
    }
    std::cout << klatka;
}