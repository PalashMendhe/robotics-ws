#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "queue"

class SensorSubscriber : public rclcpp::Node
{
public:
SensorSubscriber() : Node("SensorSubscriber")
{
subscription_ = this->create_subscription<std_msgs::msg::Float64>("sensor_data", 10, 
		std::bind(&SensorSubscriber::topic_callback, this, std::placeholders::_1));
}
private:
    std::queue<double> reading_queue_;
    double running_sum = 0.0;
    int messageCount_ = 0;
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
{

double current_reading = msg -> data;
reading_queue_.push(current_reading);
running_sum += current_reading;

if(reading_queue_.size() > 10){
running_sum -= reading_queue_.front();
reading_queue_.pop();
}
double avg = running_sum/reading_queue_.size();

messageCount_++;
if(messageCount_ % 5 == 0){
RCLCPP_INFO(this -> get_logger(), "Running Average: %.2f | Current Reading: %.2f", avg, current_reading);
}
if(current_reading > 3.5){
RCLCPP_WARN(this -> get_logger(), "Value above threshold: %.2f", current_reading);
}


}
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
