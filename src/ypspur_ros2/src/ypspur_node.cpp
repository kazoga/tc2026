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
    vmax_linear_       = declare_parameter<double>("velocity_max.linear", 1.0);
    vmax_angular_      = declare_parameter<double>("velocity_max.angular", 1.5);

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
      if (YPSpur_init() < 0) {
        RCLCPP_FATAL(get_logger(),
                     "YPSpur_init failed. Is ypspur-coordinator running?");
        throw std::runtime_error("YPSpur_init failed");
      }
    }

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

    last_cmd_vel_time_ = now();
    cmd_vel_active_ = false;

    RCLCPP_INFO(get_logger(),
                "ypspur_node ready (timeout=%.2fs, odom=%.0fHz, cs=%d)",
                cmd_vel_timeout_s_, odom_publish_hz_, coordinate_system_);
  }

  ~YpspurNode() override
  {
    try {
      YPSpur_vel(0.0, 0.0);
      YPSpur_free();
    } catch (...) {
      // best-effort cleanup
    }
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double v = clamp(msg->linear.x,  vmax_linear_);
    const double w = clamp(msg->angular.z, vmax_angular_);
    if (YPSpur_vel(v, w) < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "YPSpur_vel failed (coordinator down?)");
    }
    last_cmd_vel_time_ = now();
    cmd_vel_active_ = true;
  }

  void watchdog_tick()
  {
    if (!cmd_vel_active_) return;
    const auto elapsed = (now() - last_cmd_vel_time_).seconds();
    if (elapsed > cmd_vel_timeout_s_) {
      YPSpur_vel(0.0, 0.0);
      cmd_vel_active_ = false;
      RCLCPP_INFO(get_logger(),
                  "cmd_vel timeout (%.2fs), stopping robot", elapsed);
    }
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

  // pubsub
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // state
  rclcpp::Time last_cmd_vel_time_;
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
