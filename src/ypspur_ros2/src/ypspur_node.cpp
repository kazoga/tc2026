// ypspur_node — geometry_msgs/Twist の cmd_vel を購読し、
// YPSpur_vel() でロボットを駆動。 YPSpur_get_pos/_get_vel から
// nav_msgs/Odometry を配信。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tc_route_msgs/msg/motion_limits.hpp>
#include "motion_profile.hpp"

extern "C" {
#include <ypspur.h>
}

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace
{
double clamp(double v, double limit)
{
  if (limit <= 0.0) return v;
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}
}  // namespace

class YpspurNode : public rclcpp::Node
{
public:
  YpspurNode()
  : Node("ypspur_node")
  {
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);
    odom_publish_hz_   = declare_parameter<double>("odom_publish_hz", 50.0);
    odom_frame_id_     = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_     = declare_parameter<std::string>("base_frame_id", "base_link");
    coordinate_system_ = declare_parameter<int>("coordinate_system", static_cast<int>(CS_GL));
    use_socket_        = declare_parameter<bool>("ipc.use_socket", false);
    ipc_ip_            = declare_parameter<std::string>("ipc.ip", "127.0.0.1");
    ipc_port_          = declare_parameter<int>("ipc.port", 54321);
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    vmax_linear_       = declare_parameter<double>("velocity_max.linear", 1.0, immutable);
    vmax_angular_      = declare_parameter<double>("velocity_max.angular", 1.0, immutable);

    accel_linear_ = declare_parameter<double>("acceleration_max.linear", 0.7, immutable);
    decel_linear_ = declare_parameter<double>("deceleration_max.linear", 1.5, immutable);
    accel_angular_ = declare_parameter<double>("acceleration_max.angular", 0.6, immutable);
    for (double limit : {vmax_linear_, vmax_angular_, accel_linear_, accel_angular_, decel_linear_}) {
      if (!std::isfinite(limit) || limit <= 0.0) {
        throw std::runtime_error("Velocity and acceleration limits must be finite and positive");
      }
    }

    if (use_socket_) {
      RCLCPP_INFO(get_logger(), "Connecting to ypspur-coordinator via TCP %s:%d",
                  ipc_ip_.c_str(), ipc_port_);
      // YPSpur_init_socket は char* を取るので const を剥がす
      char ip_buf[256];
      std::snprintf(ip_buf, sizeof(ip_buf), "%s", ipc_ip_.c_str());
      if (YPSpur_init_socket(ip_buf, ipc_port_) < 0) {
        RCLCPP_FATAL(get_logger(), "YPSpur_init_socket failed");
        throw std::runtime_error("YPSpur_init_socket failed");
      }
    } else {
      RCLCPP_INFO(get_logger(), "Connecting to ypspur-coordinator via local IPC");
      const int ipc_key = declare_parameter<int>("ipc.key", 28741, immutable);
      if (YPSpur_initex(ipc_key) < 0) {
        RCLCPP_FATAL(get_logger(),
                     "YPSpur_init failed. Is ypspur-coordinator running?");
        throw std::runtime_error("YPSpur_init failed");
      }
    }

    YPSpur_vel(0.0, 0.0);
    // Verify the loaded coordinator, not just a presumed YAML or .param path.
    // Abort before accepting commands if it would silently clamp the brake rate.
    const auto check = [](int id, double requested) {
      double ceiling = 0.0;
      if (YP_get_parameter(id, &ceiling) != id) {
        throw std::runtime_error("Cannot read coordinator motion limits");
      }
      icart_motion::validate_limit(requested, ceiling);
    };
    check(YP_PARAM_MAX_VEL, vmax_linear_);
    check(YP_PARAM_MAX_W, vmax_angular_);
    check(YP_PARAM_MAX_ACC_V, accel_linear_);
    check(YP_PARAM_MAX_ACC_V, decel_linear_);
    check(YP_PARAM_MAX_ACC_W, accel_angular_);

    // Coordinator user limits start at zero; cmd_vel alone cannot move the robot.
    // Clear a previous client's target before enabling nonzero motion limits.
    if (YPSpur_vel(0.0, 0.0) < 0 ||
        YPSpur_set_vel(vmax_linear_) < 0 ||
        YPSpur_set_angvel(vmax_angular_) < 0 ||
        YPSpur_set_accel(decel_linear_) < 0 ||
        YPSpur_set_angaccel(accel_angular_) < 0) {
      throw std::runtime_error("Failed to initialize YP-Spur motion limits");
    }

    limits_pub_ = create_publisher<tc_route_msgs::msg::MotionLimits>("motion_limits", 1);
    control_timer_ = create_wall_timer(20ms, std::bind(&YpspurNode::control_tick, this));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&YpspurNode::on_cmd_vel, this, _1));

    const auto odom_period =
      std::chrono::duration<double>(1.0 / std::max(odom_publish_hz_, 1.0));
    odom_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(odom_period),
      std::bind(&YpspurNode::publish_odom, this));

    watchdog_timer_ = create_wall_timer(
      100ms, std::bind(&YpspurNode::watchdog_tick, this));

    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    cmd_vel_active_ = false;

    RCLCPP_INFO(get_logger(),
                "ypspur_node ready (timeout=%.2fs, odom=%.0fHz, cs=%d)",
                cmd_vel_timeout_s_, odom_publish_hz_, coordinate_system_);
  }

  ~YpspurNode() override
  {
    try {
      YPSpur_set_accel(decel_linear_);
      YPSpur_vel(0.0, 0.0);
      YPSpur_free();
    } catch (...) {
      // best-effort cleanup
    }
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      cmd_vel_active_ = false;
      stop();
      return;
    }
    target_linear_ = clamp(msg->linear.x, vmax_linear_);
    target_angular_ = clamp(msg->angular.z, vmax_angular_);
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    cmd_vel_active_ = true;
  }

  void stop()
  {
    target_linear_ = target_angular_ = 0.0;
    YPSpur_vel(0.0, 0.0);
    YPSpur_set_accel(decel_linear_);
  }

  void watchdog_tick()
  {
    if (!cmd_vel_active_) return;
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_cmd_vel_time_).count();
    if (elapsed > cmd_vel_timeout_s_) {
      stop();
      cmd_vel_active_ = false;
      RCLCPP_INFO(get_logger(), "cmd_vel timeout (%.2fs), stopping robot", elapsed);
    }
  }

  void control_tick()
  {
    watchdog_tick();
    double reference = 0.0, angular_reference = 0.0;
    // The smoothed reference is what the coordinator actually slews. Checking
    // just the previous Twist misclassifies repeated stop/reverse requests.
    if (YP_get_vref(&reference, &angular_reference) < 0.0 || !std::isfinite(reference)) {
      stop();
      return;  // No healthy limits heartbeat; navigator must stop too.
    }
    const auto command = icart_motion::select_linear_command(
      reference, cmd_vel_active_ ? target_linear_ : 0.0, accel_linear_, decel_linear_);
    // When braking, lower the target BEFORE increasing the slew rate, so
    // an intervening coordinator cycle cannot accelerate the old target at 1.5.
    // When accelerating, lower the slew rate BEFORE raising the target.
    const double angular = cmd_vel_active_ ? target_angular_ : 0.0;
    const bool braking = command.target == 0.0 ||
      std::abs(command.target) < std::abs(reference);
    const bool failed = braking ?
      (YPSpur_vel(command.target, angular) < 0 || YPSpur_set_accel(command.acceleration) < 0) :
      (YPSpur_set_accel(command.acceleration) < 0 || YPSpur_vel(command.target, angular) < 0);
    if (failed) {
      stop();
      return;
    }
    tc_route_msgs::msg::MotionLimits limits;
    limits.header.stamp = now();
    limits.max_linear_velocity = vmax_linear_;
    limits.max_angular_velocity = vmax_angular_;
    limits.linear_acceleration = accel_linear_;
    limits.linear_deceleration = decel_linear_;
    limits.angular_acceleration = accel_angular_;
    limits_pub_->publish(limits);
  }

  void publish_odom()
  {
    double x = 0.0, y = 0.0, th = 0.0;
    double v = 0.0, w = 0.0;
    YPSpur_get_pos(static_cast<YPSpur_cs>(coordinate_system_), &x, &y, &th);
    YPSpur_get_vel(&v, &w);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(th * 0.5);
    odom.pose.pose.orientation.w = std::cos(th * 0.5);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    // 共分散は粗い既定値 (要 calibration)。対角に小さめの分散を入れる。
    odom.pose.covariance[0]  = 1e-3;   // x
    odom.pose.covariance[7]  = 1e-3;   // y
    odom.pose.covariance[35] = 1e-2;   // yaw
    odom.twist.covariance[0]  = 1e-3;  // vx
    odom.twist.covariance[35] = 1e-2;  // wz
    odom_pub_->publish(odom);
  }

  // params
  double cmd_vel_timeout_s_;
  double odom_publish_hz_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  int coordinate_system_;
  bool use_socket_;
  std::string ipc_ip_;
  int ipc_port_;
  double vmax_linear_;
  double vmax_angular_;

  double accel_linear_;
  double decel_linear_;
  double accel_angular_;
  double target_linear_ = 0.0;
  double target_angular_ = 0.0;

  // pubsub
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<tc_route_msgs::msg::MotionLimits>::SharedPtr limits_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // state
  std::chrono::steady_clock::time_point last_cmd_vel_time_;
  bool cmd_vel_active_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<YpspurNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ypspur_node"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
