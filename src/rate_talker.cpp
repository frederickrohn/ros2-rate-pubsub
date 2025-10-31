#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp" //this is the message type that the talker will publish
#include <mutex> //lock for rebuilding timer
#include <vector> // for parameter callback
#include <rcl_interfaces/msg/set_parameters_result.hpp> //for parameter callback

class RateTalker : public rclcpp::Node {
    public:
        RateTalker():rclcpp::Node("rate_talker"){

            //get the parameter and make it a ROS2 parameter
            publish_hz_ = this->declare_parameter<double>("publish_hz", 10.0);

            //clamp it to a valid value
            if(publish_hz_ <= 0.0){ 
                RCLCPP_WARN(this->get_logger(), "publish_hz parameter must be > 0.0, defaulting to 10.0");
                publish_hz_ = 10.0;
            }

            //create publisher
            pub_ = this->create_publisher<std_msgs::msg::Float32>("heartbeat", 10); //topic name, queue size

            make_timer();

            on_set_param_handle_ = this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> & params){
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;

                        for (const auto & p : params) { //loop through params till we find the one we care about
                            if (p.get_name() == "publish_hz") { // we found it
                                    double new_hz = p.as_double(); //
                                    if (new_hz <= 0.0) { // invalid parameter
                                    result.successful = false;
                                    result.reason = "publish_hz must be > 0.0";
                                    return result;
                                    }
                                    //set the parameter and rebuild timer
                                    publish_hz_ = new_hz;
                                    rebuild_timer();  // cancel old, make new
                                    RCLCPP_INFO(this->get_logger(), "publish_hz updated to %.2f", publish_hz_);
                                }
                            }
                            return result;
                }
            );

        };
    private:
        //creates timer
        void make_timer(){
            using std::chrono::duration;
            using std::chrono::duration_cast;
            using std::chrono::nanoseconds;

            auto period = duration_cast<nanoseconds>(duration<double>(1.0/publish_hz_));

            timer_ = this->create_wall_timer(period, [this](){
                std_msgs::msg::Float32 message;
                message.data = counter_;
                counter_++;
                pub_->publish(message);
            });
        };

        //rebuilds timer
        void rebuild_timer(){
            std::lock_guard<std::mutex> lock(timer_mtx_);
            if(timer_) timer_->cancel();
            make_timer();
        };

        // state stuff below
        double publish_hz_{10.0};     // parameter (default)
        float counter_{0.0f};         // message payload (increments)
        std::mutex timer_mtx_;      //mutex to protect timer during rebuild
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_handle_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RateTalker>());
    rclcpp::shutdown();
    return 0;
};