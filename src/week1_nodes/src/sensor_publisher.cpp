#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SensorPublisher : public rclcpp::Node {
public:
    SensorPublisher() : Node("sensor_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "sensor_data", 10);

	this -> declare_parameters("publish_frequnecy", 500);
	this -> declare_parameters("noise_amplitude", 0.1);
	this -> declare_parameters("warning_threshold", 3.5);

	int freq = this->get_parameters("publish_frequency").as_int();
	amplitude_ = this->get_parameters("noise_amplitude").as_double();
	threshold_ = this->get_parameters("warning_threshold").as_double();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(freq),
            std::bind(&SensorPublisher::publishData, this));
	param_callback_ = this->add_on_set_parameters_callback(
    		[this](const std::vector<rclcpp::Parameter>& params) {
        		for (const auto& param : params) {
		            if (param.get_name() == "warning_threshold") {
                		threshold_ = param.as_double();
		                RCLCPP_INFO(this->get_logger(), 
                    		"Threshold updated to: %.2f", threshold_);
            }
        }
        return rcl_interfaces::msg::SetParametersResult{};
    });

        RCLCPP_INFO(this->get_logger(), "Sensor publisher started");
    }

private:
    void publishData() {
        auto msg = std_msgs::msg::Float64();
        msg.data = 3.0 + 0.1 * (count_ % 10);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: %.2f", msg.data);
        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr noise_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr threshold;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisher>());
    rclcpp::shutdown();
    return 0;
}


