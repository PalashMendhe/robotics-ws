#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include <cmath>

class TF2BroadcasterNode : public rclcpp::Node {
public:
    TF2BroadcasterNode() : Node("tf2_broadcaster") {
        // Initialize broadcasters
        static_broadcaster_ = 
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        dynamic_broadcaster_ = 
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Broadcast static transforms once
        broadcastStaticTransforms();

        // Broadcast dynamic transform every 100ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TF2BroadcasterNode::broadcastDynamicTransform, this));

        RCLCPP_INFO(this->get_logger(), "TF2 broadcaster started");
    }

private:
    void broadcastStaticTransforms() {
        geometry_msgs::msg::TransformStamped lidar_tf;
	geometry_msgs::msg::TransformStamped camera_tf;
	
	lidar_tf.header.stamp = this->now();
	lidar_tf.header.frame_id = "base_link";
	lidar_tf.child_frame_id = "lidar_link";

	lidar_tf.transform.translation.x = 0.0;
	lidar_tf.transform.translation.y = 0.0;
	lidar_tf.transform.translation.z = 0.2;

	camera_tf.header.stamp = this->now();
	camera_tf.header.frame_id = "base_link";
	camera_tf.child_frame_id = "camera_link";

	camera_tf.transform.translation.x = 0.15;
	camera_tf.transform.translation.y = 0.0;
	camera_tf.transform.translation.z = 0.25;

	tf2::Quaternion q_lidar;
	q_lidar.setRPY(0, 0, 0);
	lidar_tf.transform.rotation.x = q_lidar.x();
	lidar_tf.transform.rotation.y = q_lidar.y();
	lidar_tf.transform.rotation.z = q_lidar.z();
	lidar_tf.transform.rotation.w = q_lidar.w();

	tf2::Quaternion q_camera;
	q_camera.setRPY(0, -0.523, 0);
	camera_tf.transform.rotation.x = q_camera.x();
        camera_tf.transform.rotation.y = q_camera.y();
        camera_tf.transform.rotation.z = q_camera.z();
        camera_tf.transform.rotation.w = q_camera.w();

	static_broadcaster_->sendTransform(lidar_tf);
	static_broadcaster_->sendTransform(camera_tf);

    }

    void broadcastDynamicTransform() {
        geometry_msgs::msg::TransformStamped base_tf;
	base_tf.header.stamp = this->now();
	base_tf.header.frame_id = "world";
	base_tf.child_frame_id = "base_link";

	double t = this->now().seconds();
	base_tf.transform.translation.x = cos(t) * 1.0;
	base_tf.transform.translation.y = sin(t) * 1.0;
	base_tf.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0, 0, t);
        base_tf.transform.rotation.x = q.x();
        base_tf.transform.rotation.y = q.y();
        base_tf.transform.rotation.z = q.z();
        base_tf.transform.rotation.w = q.w();

        dynamic_broadcaster_->sendTransform(base_tf);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TF2BroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}

