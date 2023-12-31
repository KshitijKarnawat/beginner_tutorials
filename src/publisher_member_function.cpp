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

/**
 * @file publisher_member_function.cpp
 * @author Kshitij Karnawat (kshitij@terpmail.umd.edu)
 * @brief Publisher with services.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <beginner_tutorials/srv/string_change.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief Class for Publisher
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Set the publisher frequency
    // Parmeter
    auto parameter_description = rcl_interfaces::msg::ParameterDescriptor();
    parameter_description.description = "Set publisher frequency.";
    this->declare_parameter("pub_freq", 1.0, parameter_description);
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Publishing frequency is set to 1.0 hz");

    parameter_event_handler_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_ = parameter_event_handler_->add_parameter_callback(
        "pub_freq", std::bind(&MinimalPublisher::parameter_callback, this,
                              std::placeholders::_1));

    // Publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publisher Created");

    // Timer
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    // Service
    client_ = this->create_client<beginner_tutorials::srv::StringChange>(
        "string_change_service");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Client created");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL_STREAM(this->get_logger(),
                            "Service Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN_STREAM(this->get_logger(), "Service Unavailable");
    }

    // Broadcaster
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.1;
    t.transform.translation.z = 0.1;

    t.transform.rotation.x = 0.141592;
    t.transform.rotation.y = 0.141592;
    t.transform.rotation.z = 0.141592;
    t.transform.rotation.w = 0.5;

    tf_static_broadcaster_->sendTransform(t);
  }

 private:
  /**
   * @brief timer callback function
   *
   */
  void timer_callback() {
    message.data = "Go Terps!!" + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
    if (count_ % 10 == 0) {
      service_call();
    }
  }

  /**
   * @brief calls the service
   *
   */
  void service_call() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Service Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN_STREAM(this->get_logger(), "Service Unavailable");
    }
    auto request =
        std::make_shared<beginner_tutorials::srv::StringChange::Request>();
    request->input = "String Changed!!";
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Serivce Called");
    client_->async_send_request(
        request, std::bind(&MinimalPublisher::service_callback, this,
                           std::placeholders::_1));
    // return result.get()->output;
  }

  /**
   * @brief Service callback funtion
   *
   * @param future
   */
  void service_callback(
      rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedFuture
          future) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Got string: " << future.get()->output);
    message.data = future.get()->output + std::to_string(count_++);
    publisher_->publish(message);
  }

  /**
   * @brief Parameter callback server. Adjusts the parameters
   *
   * @param parameter
   */
  void parameter_callback(const rclcpp::Parameter &parameter) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Parameter " << parameter.get_name() << "updated.");

    RCLCPP_FATAL_EXPRESSION(
        this->get_logger(), parameter.as_double() == 0.0,
        "Frequency cannot be set to zero due to zero division error");
    if (parameter.as_double() == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Frequency has not been changed.");
    } else {
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(
              static_cast<int>((1000 / parameter.as_double()))),
          std::bind(&MinimalPublisher::timer_callback, this));
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedPtr client_;
  std_msgs::msg::String message;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
