#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <vector>
#include <functional>
#include <memory>
#include <chrono>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>
#include <algorithm>


struct Waypoint {
            double x, y, yaw;
        };

class WaypointNavigatorNode : public rclcpp::Node {
    public:
    
        WaypointNavigatorNode() : Node("waypoint_navigator_node"),
        current_pose_x_(0.0),
        current_pose_y_(0.0),
        current_pose_yaw_(0.0),
        current_waypoint_index_(0) 
        {
            //Subscribe to for current pose and publish navigation commands
            waypoint_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered", 10, std::bind(&WaypointNavigatorNode::waypointCallback, this, std::placeholders::_1));
            waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/model/my_robot/cmd_vel", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&WaypointNavigatorNode::navigate, this));
            RCLCPP_INFO(this->get_logger(), "Waypoint Navigator Node started");

            waypoints_ = {
                {1.0, 0.0, 0.0},
                {1.0, 1.0, 1.57},
                {0.0, 1.0, 3.14},
                {0.0, 0.0, 0.0}
            };

        }
    private:

        void waypointCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            current_pose_x_ = msg->pose.pose.position.x;
            current_pose_y_ = msg->pose.pose.position.y;

            {
                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double qz = msg->pose.pose.orientation.z;
                double qw = msg->pose.pose.orientation.w;
                double siny_cosp = 2.0 * (qw * qz + qx * qy);
                double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                current_pose_yaw_ = std::atan2(siny_cosp, cosy_cosp);
            }
        }

        void navigate(){
            if  (current_waypoint_index_ >= waypoints_.size()){
                RCLCPP_INFO(this->get_logger(), "all waypoints reached");
                return;
            }
            Waypoint target_waypoint = waypoints_[current_waypoint_index_];
            double distance = sqrt(pow(target_waypoint.x - current_pose_x_, 2) + pow(target_waypoint.y - current_pose_y_, 2));
            double angle_to_goal= atan2(target_waypoint.y - current_pose_y_, target_waypoint.x - current_pose_x_);

            double heading_error = atan2(sin(angle_to_goal - current_pose_yaw_), cos(angle_to_goal - current_pose_yaw_));

            if (distance < 0.1){
                RCLCPP_INFO(this->get_logger(), "waypoint %zu reached", current_waypoint_index_);
                current_waypoint_index_++;
            }
            else if(fabs(heading_error) > 0.1){
                
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.angular.z = std::clamp(Ka * heading_error, -max_angular_vel, max_angular_vel);
                cmd_vel.linear.x = 0;

                waypoint_publisher_->publish(cmd_vel);
            }
            else{
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.angular.z = std::clamp(Ka * heading_error, -max_angular_vel, max_angular_vel);
                cmd_vel.linear.x = std::min(Kl * distance, max_linear_vel);
                waypoint_publisher_->publish(cmd_vel);
            }
        }


        
        //member variables
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr waypoint_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr waypoint_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        double current_pose_x_;
        double current_pose_y_;
        double current_pose_yaw_;
        double Ka = 1.0; // angular gain
        double Kl = 0.5; // linear gain
        double max_angular_vel = 0.5; // max angular velocity
        double max_linear_vel = 0.3; // max linear velocity
        
        std::vector<Waypoint> waypoints_;
        size_t current_waypoint_index_;
         
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}