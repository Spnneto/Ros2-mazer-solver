#define _USE_MATH_DEFINES
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <cmath>
#include <algorithm>
using namespace std::chrono_literals;
using std::placeholders::_1;
enum class RobotState {
    START,
    MOVE_FORWARD,
    ADVANCE,
    TURN_LEFT,
    TURN_RIGHT,
    STOP
};
class RobotControl : public rclcpp::Node {
    private:
        RobotState current_state_ { RobotState::START };
        double yaw_ {0.0}, goal_yaw_ {0.0};
        double advance_start_x_ {0.0}, advance_start_y_{0.0};
        double move_start_x_ {0.0}, move_start_y_ {0.0};
        double ignore_left_dist_ {0.0};
        double ADVANCE_DIST {0.45};
        const double KP = 0.8, ANGLE_TOL = 0.01;
        double front_sensor_ {10.0};
        double left_sensor_ {10.0};
        double pos_x_ {0.0};
        double pos_y_{0.0};
        double angu_vel {0.0};
        double linear_accel {0.0};
        double last_y {0.0};        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
    RobotControl() : Node("robot_control")
    {
        this->declare_parameter<double>("last_y", 10.0);
        this->declare_parameter<int>("update_frequency", 0);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RobotControl::odom_callback, this, _1));
        scan_sub = this ->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&RobotControl::scan_callback, this, _1));
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&RobotControl::imu_callback, this, _1 ));
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&RobotControl::timer_callback, this));
    }
    void timer_callback()
    {
        
        auto msg_vel = geometry_msgs::msg::Twist();
        if(pos_y_ >= 10) {
            current_state_ = RobotState::STOP;
        }
        switch(current_state_){
            case RobotState::START:
                RCLCPP_INFO(this->get_logger(), "Iniciando FSM...");
                if(front_sensor_ < 0.6){
                    start_turn(false);
                    current_state_ = RobotState::TURN_RIGHT;
                } else{
                    msg_vel.linear.x = 0.2;
                    msg_vel.angular.z = 0;
                }
                break;
            case RobotState::MOVE_FORWARD: {
                double moved_dist = std::hypot(pos_x_ - move_start_x_, pos_y_ - move_start_y_);
                bool can_check_left = (moved_dist >= ignore_left_dist_);

                if(front_sensor_ < 0.50){
                    if(can_check_left && left_sensor_ > 0.85){
                        start_turn(true);
                        current_state_ = RobotState::TURN_LEFT;
                    } else {
                        start_turn(false);
                        current_state_ = RobotState::TURN_RIGHT;
                    }
                    break;
                }

                if(can_check_left && left_sensor_ > 0.85){
                    advance_start_x_ = pos_x_;
                    advance_start_y_ = pos_y_;
                    current_state_ = RobotState::ADVANCE;
                    break;
                }

                msg_vel.linear.x = 0.2;
                double err_yaw = normalize(goal_yaw_ - yaw_);
                msg_vel.angular.z = std::clamp(KP * err_yaw, -0.5, 0.5); 
                break;
            }
            case RobotState::TURN_LEFT:
            case RobotState::TURN_RIGHT: {
                double err = normalize(goal_yaw_ - yaw_); 
                if(std::abs(err) < ANGLE_TOL){
                    move_start_x_ = pos_x_;
                    move_start_y_ = pos_y_;
                    ignore_left_dist_ = (current_state_ == RobotState::TURN_LEFT) ? 0.60 : 0.15;
                    current_state_ = RobotState::MOVE_FORWARD;
                } else {
                    msg_vel.linear.x = 0.0;
                    msg_vel.angular.z = std::clamp(KP * err, -1.0, 1.0);    
                }
                break;
            }
            case RobotState::ADVANCE: {
                 double dist = std::hypot(pos_x_ - advance_start_x_, pos_y_ - advance_start_y_);
                RCLCPP_INFO(this->get_logger(), "ADVANCE dist=%.2f", dist);
                if (dist >= ADVANCE_DIST || front_sensor_ < 0.5) {
                    start_turn(true);
                    current_state_ = RobotState::TURN_LEFT;
                } else {
                    msg_vel.linear.x = 0.2;
                    double err_yaw = normalize(goal_yaw_ - yaw_);
                    msg_vel.angular.z = std::clamp(KP * err_yaw, -0.5, 0.5);    }
                break;
            }
                 
            
            case RobotState::STOP:
                msg_vel.linear.x = 0.0;
                msg_vel.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "OBJETIVO CONCLUIDO!");
                break; }
        cmd_vel_pub->publish(msg_vel);
}
    double normalize(double a) { return std::atan2(std::sin(a), std::cos(a));}
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pos_x_ = msg->pose.pose.position.x;
        pos_y_= msg->pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
    }
    void start_turn(bool left) {
        double start = yaw_;
        double snapped = (M_PI/2) * std::round(start / (M_PI/2));
        double delta = left ? M_PI/2 : -M_PI/2;
        goal_yaw_ = normalize(delta + snapped);

    }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if(!msg->ranges.empty()) {
            front_sensor_ = msg->ranges[0];
            left_sensor_ = msg->ranges[1];
    }
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
 {
    angu_vel = msg->angular_velocity.z;
    linear_accel = msg->linear_acceleration.x;
}
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControl>());
    rclcpp::shutdown();
    return 0;
}
