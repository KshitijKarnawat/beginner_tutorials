#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/string_change.hpp"

#include <memory>

void stringChanger(const std::shared_ptr<beginner_tutorials::srv::StringChange::Request> request,
          std::shared_ptr<beginner_tutorials::srv::StringChange::Response> response)
{
  response->output = request->input;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Incoming request: " << request->input);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "sending back response: " << response->output);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("change_string");

  rclcpp::Service<beginner_tutorials::srv::StringChange>::SharedPtr service =
    node->create_service<beginner_tutorials::srv::StringChange>("string_change_service", &stringChanger);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Server Ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}