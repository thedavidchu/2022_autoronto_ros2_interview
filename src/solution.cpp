/// This is the code for my application to aUToronto's ROS2 Team. 
/// References:
/// 1. Publisher: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
/// 2. Subscriber: https://github.com/ros2/examples/blob/humble/rclcpp/topics/minimal_subscriber/lambda.cpp
#include <cassert>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "find_indices.hpp"
#include "ivec2str.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

/// Assumptions:
/// 1. The dimensions of "/input" are meaningless.
class Solution : public rclcpp::Node {
public:
    Solution() : Node("solution") {
        // Subscriber
        input_subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
            "/input", 10, std::bind(&Solution::input_callback, this, _1));
        target_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
            "/target", 10, std::bind(&Solution::target_callback, this, _1));

        input_is_valid_ = false;
        target_is_valid_ = false;

        // Publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/solution", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&Solution::timer_callback, this));
    }

private:
    void input_callback(const std_msgs::msg::Int8MultiArray &input) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to '/input': %s",
                ivec2str(input.data).c_str());
        assert(!input_is_valid_ && "clobbering input!");
        input_ = input.data;
        input_is_valid_ = true;
    }
    
    void target_callback(const std_msgs::msg::Int8 &target) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to '/target': %d",
                static_cast<int>(target.data));
        assert(!target_is_valid_ && "clobbering target!");
        target_ = target.data;
        target_is_valid_ = true;
    } 

    void timer_callback() {
        auto solution = std_msgs::msg::Int8MultiArray();

        // Error handling
        if (!input_is_valid_ || !target_is_valid_) {
            static int counter = 0;
            RCLCPP_INFO(this->get_logger(), "Publishing to '/solution': nothing! (x%d)", counter++);
            publisher_->publish(solution);
            return;
        }

        input_is_valid_ = false;
        target_is_valid_ = false;

        solution.data = find_indices(input_, target_);
        /* std::vector */
        // NOTE: the solution does not need its dimensions specified because
        // we are only using it as a 1-D vector. Otherwise, we need to specify
        // the three attributes: u(label, size, stride).
        // solution.data = std::vector<std::int8_t>{indices.first, indices.second};   // can we return the pair directly?
        RCLCPP_INFO(this->get_logger(), "Publishing to '/solution': %s",
                ivec2str(solution.data).c_str());
        publisher_->publish(solution);
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr input_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr target_subscription_;
    // Subscription debugging
    bool input_is_valid_;
    bool target_is_valid_;
    // Subscription message passing
    std::vector<std::int8_t> input_;
    std::int8_t target_;
    // Publisher
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    // // Run Testing:
    // test_find_indices();
    // test_ivec2str();
    
    // Run ROS2 Node:
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Solution>());
    rclcpp::shutdown();
    return 0;
}
