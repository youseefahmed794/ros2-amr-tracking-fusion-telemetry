#define AMR_CONTROLLER_NO_MAIN
#include "../src/adaptive_controller_node.cpp"
#include <thread>
#include <limits>
#include <iostream>
#include <vector>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto controller = std::make_shared<amr_control::AdaptiveControllerCppNode>();
    auto probe = std::make_shared<rclcpp::Node>("cpp_controller_test_probe");
    auto odom_pub = probe->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
    std::vector<geometry_msgs::msg::Twist> commands;
    std::vector<std::string> diagnostics;
    auto commands_sub = probe->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [&](geometry_msgs::msg::Twist::SharedPtr msg) {commands.push_back(*msg);});
    auto diag_sub = probe->create_subscription<std_msgs::msg::String>("/controller/telemetry_json", 10,
      [&](std_msgs::msg::String::SharedPtr msg) {diagnostics.push_back(msg->data);});
    // A single test thread drives callbacks; deployment uses MultiThreadedExecutor.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller);
    executor.add_node(probe);
    auto spin_for = [&](double seconds, bool publish, int invalid) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline) {
          if (publish) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = probe->get_clock()->now();
            msg.pose.pose.orientation.w = 1.0;
            if (invalid == 1) {msg.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();}
            if (invalid == 2) {
              msg.header.stamp = probe->get_clock()->now() - rclcpp::Duration::from_seconds(1.0);
            }
            odom_pub->publish(msg);
          }
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      };
    spin_for(0.15, false, false);
    require(!commands.empty(), "No startup stop command");
    require(commands.back().linear.x == 0.0, "Moved before receiving state");
    commands.clear();
    spin_for(1.0, true, false);
    bool moving = false;
    for (const auto & cmd : commands) {
      require(std::abs(cmd.linear.x) <= 1.5 && std::abs(cmd.angular.z) <= 2.0, "Output limit exceeded");
      moving = moving || std::abs(cmd.linear.x) > 0;
    }
    require(moving, "No moving command from fresh state");
    require(!diagnostics.empty(), "No diagnostic JSON");
    spin_for(0.8, true, true);
    require(commands.back().linear.x == 0.0 && commands.back().angular.z == 0.0,
      "Invalid odometry kept motion alive");
    require(diagnostics.back().find("stopped") != std::string::npos, "No stopped telemetry");
    spin_for(0.2, true, false);
    require(commands.back().linear.x != 0.0, "Did not recover with fresh state");
    spin_for(0.8, false, false);
    require(commands.back().linear.x == 0.0 && commands.back().angular.z == 0.0,
      "Did not stop on stale state");
    spin_for(0.2, true, false);
    spin_for(0.8, true, 2);
    require(commands.back().linear.x == 0.0 && commands.back().angular.z == 0.0,
      "Delayed state kept motion alive");
    controller->stop_motion();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
