#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "week1_nodes/action/calibrate.hpp"
#include <thread>
#include <chrono>
using Calibrate = week1_nodes::action::Calibrate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Calibrate>;

class CalibrationServer : public rclcpp::Node {
public:
    CalibrationServer() : Node("calibration_server") {

	auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Calibrate::Goal> calibrate)
		{
			RCLCPP_INFO(this->get_logger(), "Received goal request %f, Sensor: %s", calibrate->target_value, calibrate->sensor_name.c_str());
			(void)uuid;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		};
	auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Received request to handle goal");
			(void)goal_handle;
			return rclcpp_action::CancelResponse::ACCEPT;
		};
	auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
		{
			std::thread{
				std::bind(&CalibrationServer::execute, this, std::placeholders::_1),
				goal_handle
			}.detach();
		};
	this->action_server_ = rclcpp_action::create_server<Calibrate>(
			this,
			"calibrate",
			handle_goal,
			handle_cancel,
			handle_accepted
		);
    }
private:
	rclcpp_action::Server<Calibrate>::SharedPtr action_server_;
	void execute(const std::shared_ptr<GoalHandle> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Execeuting goal");
		rclcpp::Rate loop_rate(1);
		const auto goal = goal_handle->get_goal();
		auto feedback = std::make_shared<Calibrate:: Feedback>();
		auto result = std::make_shared<Calibrate::Result>();
		std::vector<std::string> steps = {"Zeroing sensor", "Warming up", "Taking baseline", 
                      			          "Adjusting offset", "Verifying"};

		for(int i = 0; i < 5; i++){
			if(goal_handle -> is_canceling()){
				result -> success = false;
				result -> message = "Calibration cancelled";
				goal_handle->canceled(result);
				return;
			}

		feedback->progress = (i+1) * 20.0;
		feedback->current_step = steps[i];
		goal_handle->publish_feedback(feedback);
		RCLCPP_INFO(this->get_logger(), "Step %d/5: %s (%.0f%%)", i+1, steps[i].c_str(), feedback->progress);
                std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		result -> calibrated_offset = goal->target_value - 3.5;
		result -> success = true;
		result -> message = "Calibration complete for "+ goal->sensor_name;
		goal_handle->succeed(result);
	}
};

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CalibrationServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
