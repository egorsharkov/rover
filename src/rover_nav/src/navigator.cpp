#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("navigator")
  {
    safe_dist_ = declare_parameter<double>("safe_dist", 0.6);
    lin_speed_ = declare_parameter<double>("linear_speed", 0.15);
    ang_speed_ = declare_parameter<double>("angular_speed", 0.6);
    reach_tol_ = declare_parameter<double>("reach_tol", 0.3);
    lat0_ = declare_parameter<double>("lat0", 55.7717);
    lon0_ = declare_parameter<double>("lon0", 37.6961);

    waypoints_.push_back({7.0, -1.0});

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) { on_odom(m); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m) { scan_ = m; });

    timer_ = create_wall_timer(100ms, [this]() { control(); });
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr m)
  {
    x_ = m->pose.pose.position.x;
    y_ = m->pose.pose.position.y;
    const auto & q = m->pose.pose.orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_odom_ = true;
    have_gps_ = true;
  }
  double sector_min(double a_from, double a_to) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i=0; i<scan_->ranges.size();i++ )
    {
        double angle =  scan_->angle_min + i * scan_->angle_increment;
        angle=std::atan2(std::sin(angle), std::cos(angle));
        if (angle<a_from || angle > a_to)
        {
            continue;
        }
        double r = scan_->ranges[i];
        if (!std::isfinite(r) || r < scan_->range_min)
        {
            continue;
        }
        best = std::min(r,best);
    }
    return best;
  }

  void control()
  {
    if (!have_odom_  || !scan_) { return; }

    if (current_wp_idx_ >= waypoints_.size()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "All waypoints reached. Mission complete!");
      geometry_msgs::msg::Twist stop_cmd;
      cmd_pub_->publish(stop_cmd);
      return;
    }

    double goal_x = waypoints_[current_wp_idx_].first;
    double goal_y = waypoints_[current_wp_idx_].second;

    const double dx = goal_x - x_;
    const double dy = goal_y - y_;
    const double dist = std::hypot(dx, dy);

    geometry_msgs::msg::Twist cmd;

    if (dist < reach_tol_) {
      RCLCPP_INFO(get_logger(), "Waypoint %zu reached!", current_wp_idx_);
      current_wp_idx_++;  
      return;
    }

    const double heading_err = normalize(std::atan2(dy, dx) - yaw_);
    const double front = sector_min(-0.35, 0.35);
    const double left  = sector_min(0.35, 1.57);
    const double right = sector_min(-1.57, -0.35);

  switch (state_) {
  case State::GO_TO_GOAL:
    if (front < safe_dist_) {
      side_ = (left < right) ? 1 : -1;
      state_ = State::TURN;
    }else if (now() < no_wall_until_)
    {
      cmd.linear.x = lin_speed_;
      cmd.angular.z = 0.0;
    } else {
      cmd.angular.z = std::clamp(1.5 * heading_err, -ang_speed_, ang_speed_); //обрезаем скорость если слишком быстро
      cmd.linear.x = (std::fabs(heading_err) < 0.5) ? lin_speed_ : 0.0;
    }
    break;

  case State::TURN:
    cmd.angular.z = -side_ * ang_speed_;
    if (front > safe_dist_ + 0.3) { state_ = State::FOLLOW_WALL; }
    break;

  case State::FOLLOW_WALL: {
    const double wall = (side_ == 1) ? left : right;  
    if (front < safe_dist_) { state_ = State::TURN; break; }

    // держим стену сбоку на 0.5 м
    const double err = std::isfinite(wall) ? (wall - 0.5) : 1.0;
    cmd.linear.x = lin_speed_;
    cmd.angular.z = std::clamp(side_ * 1.5 * err, -ang_speed_, ang_speed_);

    if (std::fabs(heading_err) < 1.2 && sector_min(heading_err -0.35, heading_err+0.35) > safe_dist_+0.5)
    {
      state_ = State::GO_TO_GOAL;
      no_wall_until_ = now() + rclcpp::Duration::from_seconds(1.5);
    }
    break;
  }
  
}
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "GPS: x=%.2f y=%.2f | Goal: x=%.2f y=%.2f | dist=%.2f",
    x_, y_, goal_x, goal_y, dist);
    cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "pos=(%.1f,%.1f) dist=%.1f front=%.2f left=%.2f right=%.2f", x_, y_, dist, front, left, right);
  }
  double safe_dist_, lin_speed_, ang_speed_, reach_tol_;
  double x_ = 0, y_ = 0, yaw_ = 0;
  double lat0_, lon0_;
  bool have_odom_ = false;
  bool have_gps_ = false;
  std::vector<std::pair<double, double>> waypoints_;
  size_t current_wp_idx_ = 0;
  enum class State { GO_TO_GOAL, TURN, FOLLOW_WALL };
  State state_ = State::GO_TO_GOAL;
  int side_ = 1;  
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time no_wall_until_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}