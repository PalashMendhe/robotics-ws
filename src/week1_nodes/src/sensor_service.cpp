#include "rclcpp/rclcpp.hpp"
#include "week1_nodes/srv/get_sensor_reading.hpp"
#include <map>
#include <string> 
#include <memory>

class SensorService : public rclcpp::Node
{
public:
SensorService() : Node("sensor_service_node"){
	sensor_reading_["lidar"] = 3.5;
	sensor_reading_["imu"] = 0.8;
	sensor_reading_["camera"] = 45.0;

	service_ = this->create_service<week1_nodes::srv::GetSensorReading>("/get_sensor_reading", 
			std::bind(&SensorService::handleRequest, this, std::placeholders::_1,
			std::placeholders:: _2));

	}
private:
	void handleRequest(
		const std::shared_ptr<week1_nodes::srv::GetSensorReading::Request> request,
		std::shared_ptr<week1_nodes::srv::GetSensorReading::Response> response){
			std::string sensor_request_ = request->sensor_name;
			if(sensor_reading_.contains(sensor_request_)){
				double value = sensor_reading_[sensor_request_];
				response -> value = value;
				response -> is_valid = true;
				response -> message = "Sensor" + sensor_request_ + "and its value" + 
							std::to_string(value);
				RCLCPP_INFO(this->get_logger(), "Request Handled: %s", response->message.c_str());

			}
			else{
				response -> value = 0.0;
				response -> is_valid = false;
				response -> message = "Sensor requested" + sensor_request_ + "not found";
				RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
			}
		

	}
	std::map<std::string, double> sensor_reading_;
	rclcpp::Service<week1_nodes::srv::GetSensorReading>::SharedPtr service_;
};
int main (int argc, char* argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorService>());
	rclcpp::shutdown();
	return 0;
}
