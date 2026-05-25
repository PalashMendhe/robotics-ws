#include <memory>
#include <thread>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "week1_nodes/action/calibrate.hpp"

using calibrate = week1_nodes::action::Calibrate;
using GoalHandleCalibrate = rclcpp_action::ClientGoalHandle<Calibrate>;

class CalibrationClient : public rclcpp::Node
{
public:
	{
	this->client_ptr_ = rclcpp_action::create_client<Calibrate>(this, "calibrate");
	}
void send_goal(std::string sensor, int32_t target){
	if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))){
		RCLCPP_ERROR(this->get_logger(), "Action server not available.");
		return;
		}
	auto goal_msg = Calibrate::Goal();
	goal_msg.sensor_name = sensor;
	goal_msg.target_value = target;
	RCLCPP_INFO(this->get_logger(), "Sending calibration request...");
	auto send_goal_options = rclcpp_action::Client<Calibrate>::SendGoalOptions();

	send_goal_options.goal_response_callback=
	std::bind(&CalibrationClient::goal_response_callback, this, std::placeholders::_1);

	send_goal_options.goal_feedback=
	std::bind(&CalibrationClient::goal_feedback, this, std::placeholders::_1);

	send_goal_options.goal_result=
	std::bind(&CalibrationClient::goal_result, this, std::placeholders::_1);

	this -> client_ptr_->async_send_goal(goal_msg, send_goal_options);
		}
	}
private:
	rclcpp_action::Client<Calibrate>::SharedPtr client_ptr_;

	void goal_response_callback(const GoalHandleCalibrate::ShredPtr & goal_handle){
		if(!goal_handle){
			RCLCPP_ERROR(this->get_logger(), "goal was rejected by the calibrator");
		}else{
			RCLCPP_INFO(this->get_logger(), "goal was accepted by the calibrator");
		}
	}
	void goal_feedback(GoalHandleCalibrate::SharedPtr, 
		const std::shared_ptr<const Calibrate::Feedback> feedback){
		RCLCPP_INFO(this->get_logger(), "Feedback received -> step: %s | 
				Progress: %.1f%%", feedback -> current_step.c_str(),
				feedback->progress);
	}
	void goal_result(const GoalHandleCalibrate::WrappedResult & result){
		switch(result.code){
			case rclcpp_actions::ResultCode::SUCCEEDED:
				RCLCPP_INFO(this->get_logger(),"Success! %s", result.result->message.c_str());
				RCLCPP_INFO(this->get_logger(),"Calculated offset: %.4f",result.result->calibrated_offset);
				return;
			case rclcpp_actions::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(),"Calibration was aborted.");
				return;
			case rclcpp_actions::ResultCode::CANCELED:
				RCLCPPP_WARN(this->get_logger(),"Calibration was cancelled");
				return;
			default:
				RCLCPP_ERROR(this->get_logger(),"Unknown result code received");
				return;
		}
		rclcpp::shutdown();
	}

};
int main (int argc, char ** argv)
{
rclcpp::init(argc, argv);
auto node = std:: make_shared<CalibrationClient>();
node->send_goal("IMU_0", 50);
rclcpp::spin(node);
return 0;
}
