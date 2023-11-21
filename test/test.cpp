/**
 * @file test.cpp
 * @author Kshitij Karnawat (@KshitijKarnawat)
 * @brief Test file
 * @version 0.1
 * @date 2023-11-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <beginner_tutorials/srv/string_change.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include <memory>
class TaskPlanningFixture : public testing::Test {
 public:

  void SetUp() override {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("test");
    client_ = node_->create_client<beginner_tutorials::srv::StringChange>(
        "string_change");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL_STREAM(
            node_->get_logger(),
            "Interrupted while waiting for the service. Exiting.");
        exit(1);
      }
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "service not available, waiting again...");
    }
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedPtr client_;
};

TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  EXPECT_EQ(1, 1);
}


TEST_F(TestPub, testTF) {
  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped =
        tfBuffer->lookupTransform("world", "talk", rclcpp::Time(0), 5s);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failure %s ", ex.what());
  }
  ASSERT_EQ(transformStamped.header.frame_id, "world");
  ASSERT_EQ(transformStamped.child_frame_id, "talk");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}