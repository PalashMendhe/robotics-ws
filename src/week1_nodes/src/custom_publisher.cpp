#include "rclcpp/rclcpp.hpp"
#include "week1_nodes/msg/sensor_reading.hpp"

class CustomPublisher : public rclcpp::Node{
public :
	CustomPublisher() : Node("custom_publisher"), count_(0){
		LiDAR_cycle_ = this->create_publisher<week1_nodes::msg::SensorReading>("/lidar/reading", 10);
		IMU_cycle_ = this->create_publisher<week1_nodes::msg::SensorReading>("/imu/reading", 10);
		Camera_cycle_ = this->create_publisher<week1_nodes::msg::SensorReading>("/camera/reading", 100);
	
	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(500),
		std::bind(&CustomPublisher::publishData, this));

	RCLCPP_INFO(this->get_logger(), "Custom publisher started");
	}

private:
	void publishData(){
		
		rclcpp::Time current_time = this->now();
		
		auto lidar_msg = week1_nodes::msg::SensorReading();
		lidar_msg.sensor_name = "LiDAR";
		lidar_msg.header.stamp = current_time;
		lidar_msg.header.frame_id = "lidar_frame";
		lidar_msg.value = 3.5 + (count_ % 5) * 0.5; 
		lidar_msg.is_valid = (lidar_msg.value >= 0.0 && lidar_msg.value <= 10.0);

		auto imu_msg =  week1_nodes::msg::SensorReading();
		imu_msg.sensor_name = "IMU";
		imu_msg.header.stamp = current_time;
		imu_msg.header.frame_id = "imu_frame";
		imu_msg.value = -5.0 + (count_ % 10) * 1.0; 
		imu_msg.is_valid = (imu_msg.value >= -10.0 && imu_msg.value <= 10.0);

		auto camera_msg = week1_nodes::msg::SensorReading();
		camera_msg.sensor_name = "Camera";
		camera_msg.header.stamp = current_time;
		camera_msg.header.frame_id = "camera_frame";
		camera_msg.value = 20.0 + (count_ % 8) * 10.0; 
		camera_msg.is_valid = (camera_msg.value >= 0.0 && camera_msg.value <= 100.0);


		LiDAR_cycle_-> publish(lidar_msg);
		IMU_cycle_->publish(imu_msg);
		Camera_cycle_->publish(camera_msg);
		count_++;

	}
	size_t count_ = 0;
	rclcpp::Publisher<week1_nodes::msg::SensorReading>::SharedPtr LiDAR_cycle_;
	rclcpp::Publisher<week1_nodes::msg::SensorReading>::SharedPtr IMU_cycle_;
	rclcpp::Publisher<week1_nodes::msg::SensorReading>::SharedPtr Camera_cycle_;

	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomPublisher>());
    rclcpp::shutdown();
    return 0;
}

