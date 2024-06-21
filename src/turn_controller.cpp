#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd, double setpoint)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd), setpoint_(setpoint), integral_(0.0), previous_error_(0.0) {}

  double calculate(double current_value) {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - previous_time_;
    double error = setpoint_ - current_value;

    integral_ += error * elapsed_time.count();
    double derivative = (error - previous_error_) / elapsed_time.count();

    double output = (Kp_ * error) + (Ki_ * integral_) + (Kd_ * derivative);

    previous_error_ = error;
    previous_time_ = current_time;

    return output;
  }

private:
  double Kp_, Ki_, Kd_, setpoint_, integral_, previous_error_;
  std::chrono::time_point<std::chrono::steady_clock> previous_time_ = std::chrono::steady_clock::now();
};

class TurnController : public rclcpp::Node {
public:
  TurnController()
      : Node("turn_controller"), waypoints_{{1.552, -0.880}, {3.300, -0.179}, {4.760, 1.854}},
        current_waypoint_index_(0), Kp_(1.0), Ki_(0.1), Kd_(0.05), max_angular_speed_(1.0) {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10, std::bind(&TurnController::odometry_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&TurnController::control_loop, this));
    update_target_angle();
    pid_ = std::make_shared<PIDController>(Kp_, Ki_, Kd_, target_angle_);
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }

  void update_target_angle() {
    double dx = waypoints_[current_waypoint_index_].first;
    double dy = waypoints_[current_waypoint_index_].second;
    target_angle_ = std::atan2(dy, dx);
  }

  void control_loop() {
    if (current_waypoint_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints reached. Process finished.");
      rclcpp::shutdown();
      return;
    }

    double control_signal = pid_->calculate(current_yaw_);
    control_signal = std::clamp(control_signal, -max_angular_speed_, max_angular_speed_); // Cap the control signal to the max angular speed

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = control_signal;
    velocity_publisher_->publish(cmd);

    if (std::abs(target_angle_ - current_yaw_) < 0.01) {
      cmd.angular.z = 0.0;
      velocity_publisher_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu at (%.3f, %.3f). Acquiring next waypoint.", current_waypoint_index_ + 1, waypoints_[current_waypoint_index_].first, waypoints_[current_waypoint_index_].second);
      current_waypoint_index_++;
      if (current_waypoint_index_ < waypoints_.size()) {
        update_target_angle();
        pid_ = std::make_shared<PIDController>(Kp_, Ki_, Kd_, target_angle_);
      } else {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached. Process finished.");
        rclcpp::shutdown(); // Shut down the node to exit the program
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::pair<double, double>> waypoints_;
  size_t current_waypoint_index_;
  double current_yaw_ = 0.0;
  double Kp_, Ki_, Kd_;
  double max_angular_speed_;
  double target_angle_;
  std::shared_ptr<PIDController> pid_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurnController>());
  rclcpp::shutdown();
  return 0;
}