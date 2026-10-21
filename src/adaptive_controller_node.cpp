#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <vector>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_amr_nmpc_telemetry/tracker.hpp"

namespace amr_control
{
class AdaptiveControllerCppNode : public rclcpp::Node
{
public:
  AdaptiveControllerCppNode() : Node("adaptive_controller_cpp_node")
  {
    const double rate = declare_parameter("control_rate_hz", 100.0);
    timing_path_ = declare_parameter<std::string>("timing_output", "");
    if (!timing_path_.empty()) {timing_samples_.reserve(100000);}
    state_timeout_ = declare_parameter("state_timeout_sec", 0.5);
    if (!std::isfinite(rate) || rate <= 0.0 || rate > 10000.0 ||
      !std::isfinite(state_timeout_) || state_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Rate must be in (0,10000]; state timeout must be positive");
    }
    Gains gains;
    gains.kx = declare_parameter("kx", gains.kx);
    gains.ky = declare_parameter("ky", gains.ky);
    gains.ktheta = declare_parameter("ktheta", gains.ktheta);
    gains.ki_v = declare_parameter("ki_v", gains.ki_v);
    gains.integral_limit = declare_parameter("integral_limit", gains.integral_limit);
    gains.v_max = declare_parameter("v_max", gains.v_max);
    gains.omega_max = declare_parameter("omega_max", gains.omega_max);
    gains.accel_max = declare_parameter("accel_max", gains.accel_max);
    tracker_ = Tracker(gains);
    odom_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = odom_group_;
    const auto qos = rclcpp::QoS(10).reliable();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", qos,
      std::bind(&AdaptiveControllerCppNode::odom_callback, this, std::placeholders::_1), options);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
    telemetry_pub_ = create_publisher<std_msgs::msg::String>("/controller/telemetry_json", 10);
    start_time_ = get_clock()->now();
    last_time_ = start_time_;
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      std::bind(&AdaptiveControllerCppNode::control_callback, this), control_group_);
    RCLCPP_INFO(get_logger(), "C++17 controller requested rate %.1f Hz", rate);
  }

  ~AdaptiveControllerCppNode() override
  {
    if (!timing_path_.empty()) {
      std::ofstream output(timing_path_);
      output << "start_ns,duration_ns\n";
      for (const auto & sample : timing_samples_) {
        output << sample[0] << "," << sample[1] << "\n";
      }
      if (!output) {RCLCPP_ERROR(get_logger(), "Could not write timing output %s", timing_path_.c_str());}
    }
  }

  void stop_motion()
  {
    tracker_.reset();
    if (rclcpp::ok(get_node_base_interface()->get_context())) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      std_msgs::msg::String status;
      status.data = R"({"status":"stopped","v_cmd":0.0,"omega_cmd":0.0})";
      telemetry_pub_->publish(status);
    }
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    const double source_age = (now - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || source_age > 0.25 || source_age < -0.02) {
      return;
    }
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) || norm < 1e-9) {
      return;
    }
    const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
    const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (have_state_ && stamp <= state_time_) {return;}
    pose_ = {p.x, p.y, yaw};
    state_time_ = stamp;
    have_state_ = true;
  }

  struct TimingScope
  {
    std::vector<std::array<int64_t, 2>> * samples;
    std::chrono::steady_clock::time_point start;
    explicit TimingScope(std::vector<std::array<int64_t, 2>> * target) : samples(target)
    {
      if (samples) {start = std::chrono::steady_clock::now();}
    }
    ~TimingScope()
    {
      if (samples && samples->size() < 100000) {
        const auto end = std::chrono::steady_clock::now();
        samples->push_back({
          std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count(),
          std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()});
      }
    }
  };

  void control_callback()
  {
    TimingScope timing(timing_path_.empty() ? nullptr : &timing_samples_);
    const auto now = get_clock()->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;
    std::array<double, 3> pose;
    bool fresh;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double age = have_state_ ? (now - state_time_).seconds() : -1.0;
      fresh = have_state_ && age >= 0.0 && age < state_timeout_;
      pose = pose_;
    }
    // A wall timer still runs while simulated ROS time is paused: stop on dt=0.
    if (!fresh || !std::isfinite(dt) || dt <= 1e-5 || dt > 0.1) {
      stop_motion();
      return;
    }
    const double elapsed = (now - start_time_).seconds();
    const auto ref = reference(elapsed);
    const auto command = tracker_.step(pose, elapsed, dt);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = command[0];
    cmd.angular.z = command[1];
    cmd_pub_->publish(cmd);
    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{\"timestamp_sec\":" << elapsed << ",\"stamp_sec\":" << now.seconds()
         << ",\"dt_ms\":" << dt * 1000.0
         << ",\"x\":" << pose[0] << ",\"y\":" << pose[1]
         << ",\"x_ref\":" << ref[0] << ",\"y_ref\":" << ref[1]
         << ",\"v_cmd\":" << command[0] << ",\"omega_cmd\":" << command[1]
         << ",\"tracking_error_m\":" << std::hypot(ref[0] - pose[0], ref[1] - pose[1]) << "}";
    std_msgs::msg::String diagnostic;
    diagnostic.data = json.str();
    telemetry_pub_->publish(diagnostic);
  }

  std::string timing_path_;
  std::vector<std::array<int64_t, 2>> timing_samples_;
  Tracker tracker_;
  std::mutex state_mutex_;
  std::array<double, 3> pose_{};
  bool have_state_{false};
  double state_timeout_{0.5};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::CallbackGroup::SharedPtr odom_group_, control_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace amr_control

// Tests compile the same node into an in-process ROS harness.
#ifndef AMR_CONTROLLER_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<amr_control::AdaptiveControllerCppNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    node->stop_motion();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("adaptive_controller_cpp_node"), "%s", error.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#endif
