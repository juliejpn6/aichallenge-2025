#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>

namespace simple_pure_pursuit
{

using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Odometry = nav_msgs::msg::Odometry;
using PointStamped = geometry_msgs::msg::PointStamped;

// 遅延補償対応Pure Pursuitクラス
class SimplePurePursuit : public rclcpp::Node
{
public:
  SimplePurePursuit();
  ~SimplePurePursuit() = default;

private:
  void onTimer();
  bool subscribeMessageAvailable();

  // ROS インターフェース
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;

  // センサーデータ
  Odometry::SharedPtr odometry_;
  Trajectory::SharedPtr trajectory_;

  // 基本制御パラメータ
  float wheel_base_;
  float lookahead_gain_;
  float lookahead_min_distance_;
  float speed_proportional_gain_;
  bool use_external_target_vel_;
  float external_target_vel_;
  float steering_tire_angle_gain_;
  
  // 制御機能フラグ
  bool enable_predictive_control_;        // 既存：予測制御
  bool enable_delay_compensation_;        // 新規：遅延補償制御

  // PIMPLパターンによる実装の隠蔽
  // 遅延補償システム、予測平滑化システム、軌道予測システムを含む
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace simple_pure_pursuit
