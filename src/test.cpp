#include <cstdint>
#include <vector>
#include <utility>

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

class Test : public rclcpp::Node {
public:
    Test() : Node("test"), iteration_(0) {
        input_publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/input", 10);
        target_publisher_ = this->create_publisher<std_msgs::msg::Int8>("/target", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&Test::timer_callback, this));
        
        solution_subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
            "/solution", 10, std::bind(&Test::solution_callback, this, _1));
    }
private:
    void timer_callback() {
        input_timer_callback(data_[iteration_ % data_.size()].first);
        target_timer_callback(data_[iteration_ % data_.size()].second);

        ++iteration_;
    }

    void input_timer_callback(const std::vector<std::int8_t> input_vec) {
        auto input = std_msgs::msg::Int8MultiArray();
        input.data = input_vec;
        RCLCPP_INFO(this->get_logger(), "Publishing to '/input': %s",
                ivec2str(input_vec).c_str());
        input_publisher_->publish(input);
    }

    void target_timer_callback(const std::int8_t target_int) {
        auto target = std_msgs::msg::Int8();
        target.data = target_int;
        RCLCPP_INFO(this->get_logger(), "Publishing to '/target': %d",
                target_int);
        target_publisher_->publish(target);
    }

    void solution_callback(const std_msgs::msg::Int8MultiArray &solution) {
        std::vector<std::int8_t> output = solution.data;
        RCLCPP_INFO(this->get_logger(), "Subscribing to '/solution': %s",
                ivec2str(output).c_str());

        // The solution lags the iteration by 1 cycle, because we publish at a
        // rate of 1 Hz.
        test_case_find_indices_input(data_[(iteration_ - 1) % data_.size()].first,
                data_[(iteration_ - 1) % data_.size()].second, output);
    }

    std::size_t iteration_;
    const std::vector<std::pair<std::vector<std::int8_t>, std::int8_t>> data_ = {
        {{5, 5, 14, 15, 16, 17}, 10},
        {{100, 23, 45, 6, 96, 74, 5, 122, 45}, 11},
        {{100, 127, 7, 5}, 12},
        {{8, 5, 14, 29, 74}, 13},
        {{9, 67, 94, 5}, 14},
        {{34, 10, 5}, 15},
        {{11, 99, 38, 5}, 16},
        {{12, 5, 100}, 17},
        {{100, 13, 5}, 18},
    };


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr input_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr target_publisher_;

    // Subscriber
    bool solution_is_valid_;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr solution_subscription_;
};

int main(int argc, char * argv[]) {
    // Run Testing:
    test_find_indices();
    test_ivec2str();
    
    // Run ROS2 Node:
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Test>());
    rclcpp::shutdown();
    return 0;
}