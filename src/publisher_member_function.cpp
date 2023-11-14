// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <beginner_tutorials/srv/string_change.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publisher Created");

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

    client_ = this->create_client<beginner_tutorials::srv::StringChange>("string_change_service");
    RCLCPP_DEBUG(this->get_logger(), "Client created");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Service Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Service Unavailable");
    }

  }

 private:
  void timer_callback() {
    message.data = "Go Terps!!" + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
    if (count_ % 10 == 0){
      service_call();
    }
  }


  void service_call(){
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Service Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Service Unavailable");
    }
    auto request = std::make_shared<beginner_tutorials::srv::StringChange::Request>();
    request->input = "String Changed!!";
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Serivce Called");
    auto result = client_->async_send_request(request, std::bind(&MinimalPublisher::service_callback, this, std::placeholders::_1));
    // return result.get()->output;
  }

  void service_callback(rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedFuture future){
    RCLCPP_INFO_STREAM(this->get_logger(), "Got string: " << future.get()->output);
    message.data = future.get()->output + std::to_string(count_++);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedPtr client_;
  std_msgs::msg::String message;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

