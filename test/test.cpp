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
#include "rclcpp/rclcpp.hpp"

class Test : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(Test, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("test");
  auto pub =
      node_->create_publisher<std_msgs::msg::String>("topic", 10.0);

  auto num_pub = node_->count_publishers("topic");

  EXPECT_EQ(1, static_cast<int>(num_pub));
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}