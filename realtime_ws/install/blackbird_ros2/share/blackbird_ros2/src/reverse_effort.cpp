#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


class JointPub: public rclcpp::Node{
    public:
        JointPub() : Node("effort_control_example") 
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);   
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&JointPub::timer_callback, this));
            force_flag_ = true;
            rest_ = std::vector<double>(10, 0.0);
            force_ = {0.0, 4.0, 4.0, 4.0, -1.0, 0.0, 4.0, 4.0, 4.0, -1.0}; // turn left leg inward
            
        }

        void timer_callback(){
            std_msgs::msg::Float64MultiArray msg_ = std_msgs::msg::Float64MultiArray();
            if (force_flag_) msg_.data = force_;
            else msg_.data = rest_;

            pub_->publish(msg_);
            force_flag_ = !force_flag_; 
        }

    private:
        std::vector<double> rest_;
        std::vector<double> force_;
        bool force_flag_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}