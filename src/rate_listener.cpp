#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class RateListener : public rclcpp::Node{
    public:
    RateListener():rclcpp::Node("rate_listener"){
        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "heartbeat", 
            10, 
            [this](std_msgs::msg::Float32::SharedPtr message){
                RCLCPP_INFO(this->get_logger(), "I heard: %f", message->data);
            }
        );
    }
    private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RateListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
