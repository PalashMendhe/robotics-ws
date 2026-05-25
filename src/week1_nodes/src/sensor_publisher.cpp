#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SensorPublisher : public rclcpp::Node {
public:
    SensorPublisher() : Node("sensor_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "sensor_data", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SensorPublisher::publishData, this));
        
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
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisher>());
    rclcpp::shutdown();
    return 0;
}


