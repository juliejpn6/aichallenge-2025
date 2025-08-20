// simple_pure_pursuit.cpp - 周回継続対応版（trajectory切り替えでもlookaheadを維持）

#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <deque>
#include <cmath>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

// 予測平滑化用のクラス（改良版）
class SteeringSmoothing
{
private:
  std::deque<double> steering_history_;
  std::deque<double> velocity_history_;
  double last_steering_;
  rclcpp::Time last_time_;
  bool initialized_;
  
  // パラメータ
  double smoothing_factor_;
  double max_steering_rate_;
  double prediction_horizon_;
  size_t history_size_;
  
public:
  SteeringSmoothing() 
    : last_steering_(0.0), 
      initialized_(false),
      smoothing_factor_(0.75),
      max_steering_rate_(0.8),
      prediction_horizon_(2.0),
      history_size_(5)
  {
    steering_history_.resize(history_size_, 0.0);
    velocity_history_.resize(3, 0.0);
  }
  
  void setParameters(double smoothing_factor, double max_steering_rate, double prediction_horizon)
  {
    smoothing_factor_ = smoothing_factor;
    max_steering_rate_ = max_steering_rate;
    prediction_horizon_ = prediction_horizon;
  }
  
  double smoothSteering(double raw_steering, double current_velocity, rclcpp::Time current_time)
  {
    if (!initialized_) {
      last_steering_ = raw_steering;
      last_time_ = current_time;
      initialized_ = true;
      return raw_steering;
    }
    
    // 履歴更新
    steering_history_.pop_front();
    steering_history_.push_back(raw_steering);
    
    velocity_history_.pop_front();
    velocity_history_.push_back(current_velocity);
    
    // 指数移動平均による基本平滑化
    double smoothed_steering = raw_steering;
    for (int i = steering_history_.size() - 2; i >= 0; --i) {
      smoothed_steering = smoothing_factor_ * smoothed_steering + 
                         (1.0 - smoothing_factor_) * steering_history_[i];
    }
    
    // 角速度制限適用
    double dt = (current_time - last_time_).seconds();
    if (dt > 0.001 && dt < 1.0) {
      double steering_rate = (smoothed_steering - last_steering_) / dt;
      
      if (std::abs(steering_rate) > max_steering_rate_) {
        double limited_change = std::copysign(max_steering_rate_ * dt, steering_rate);
        smoothed_steering = last_steering_ + limited_change;
      }
    }
    
    // 速度適応調整
    double avg_velocity = 0.0;
    for (double v : velocity_history_) {
      avg_velocity += v;
    }
    avg_velocity /= velocity_history_.size();
    
    if (avg_velocity > 5.0) {
      double velocity_factor = std::min(1.2, avg_velocity / 10.0);
      smoothed_steering *= (0.8 + 0.2 / velocity_factor);
    }
    
    // 履歴更新
    last_steering_ = smoothed_steering;
    last_time_ = current_time;
    
    return smoothed_steering;
  }
  
  double getVibrationMagnitude() const
  {
    if (steering_history_.size() < 2) return 0.0;
    
    double sum_diff_sq = 0.0;
    for (size_t i = 1; i < steering_history_.size(); ++i) {
      double diff = steering_history_[i] - steering_history_[i-1];
      sum_diff_sq += diff * diff;
    }
    return std::sqrt(sum_diff_sq / (steering_history_.size() - 1));
  }
};

// 軌道予測用クラス（改良版）
class TrajectoryPredictor
{
private:
  double prediction_horizon_;
  double curvature_lookahead_gain_;
  
public:
  TrajectoryPredictor() : prediction_horizon_(2.5), curvature_lookahead_gain_(1.5) {}
  
  void setParameters(double prediction_horizon, double curvature_lookahead_gain)
  {
    prediction_horizon_ = prediction_horizon;
    curvature_lookahead_gain_ = curvature_lookahead_gain;
  }
  
  template<typename TrajectoryContainer>
  double calculateAdaptiveLookahead(
    double base_lookahead, 
    double current_velocity,
    const TrajectoryContainer& trajectory,
    size_t closest_idx) const
  {
    if (closest_idx >= trajectory.size() - 1) {
      return base_lookahead;
    }
    
    double local_curvature = calculateCurvature(trajectory, closest_idx);
    double curvature_factor = 1.0 / (1.0 + std::abs(local_curvature) * curvature_lookahead_gain_);
    double velocity_factor = std::sqrt(current_velocity / 10.0);
    
    return base_lookahead * curvature_factor * velocity_factor;
  }
  
private:
  template<typename TrajectoryContainer>
  double calculateCurvature(const TrajectoryContainer& trajectory, size_t idx) const
  {
    if (idx == 0 || idx >= trajectory.size() - 1) {
      return 0.0;
    }
    
    const auto& p1 = trajectory[idx - 1].pose.position;
    const auto& p2 = trajectory[idx].pose.position;
    const auto& p3 = trajectory[idx + 1].pose.position;
    
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    double cross_product = dx1 * dy2 - dy1 * dx2;
    double norm1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double norm2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (norm1 > 0.01 && norm2 > 0.01) {
      return 2.0 * cross_product / (norm1 * norm2 * (norm1 + norm2));
    }
    
    return 0.0;
  }
};

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  // 周回継続のための新しいメンバー変数
  last_valid_trajectory_(),
  last_trajectory_timestamp_(this->now()),
  trajectory_timeout_duration_(std::chrono::milliseconds(500)),  // 500ms timeout
  use_last_valid_trajectory_(false)
{
  // 新しい予測平滑化パラメータ
  auto smoothing_factor = declare_parameter<float>("steering_smoothing_factor", 0.75);
  auto max_steering_rate = declare_parameter<float>("max_steering_rate", 0.8);
  auto prediction_horizon = declare_parameter<float>("prediction_horizon", 2.5);
  auto curvature_lookahead_gain = declare_parameter<float>("curvature_lookahead_gain", 1.5);
  auto enable_predictive_control = declare_parameter<bool>("enable_predictive_control", true);
  
  // 周回継続パラメータ
  auto trajectory_timeout_ms = declare_parameter<int>("trajectory_timeout_ms", 500);
  trajectory_timeout_duration_ = std::chrono::milliseconds(trajectory_timeout_ms);
  
  // 予測平滑化クラスの初期化
  steering_smoother_ = std::make_unique<SteeringSmoothing>();
  trajectory_predictor_ = std::make_unique<TrajectoryPredictor>();
  
  steering_smoother_->setParameters(smoothing_factor, max_steering_rate, prediction_horizon);
  trajectory_predictor_->setParameters(prediction_horizon, curvature_lookahead_gain);
  
  enable_predictive_control_ = enable_predictive_control;
  
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { 
      trajectoryCallback(msg); 
    });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));

  RCLCPP_INFO(get_logger(), "Enhanced Pure Pursuit initialized - Predictive Control: %s, Lap Continuity: ENABLED", 
              enable_predictive_control_ ? "ENABLED" : "DISABLED");
}

void SimplePurePursuit::trajectoryCallback(const Trajectory::SharedPtr msg)
{
  trajectory_ = msg;
  last_trajectory_timestamp_ = this->now();
  
  // 有効なtrajectoryを受信したので、バックアップとして保存
  if (!msg->points.empty()) {
    last_valid_trajectory_ = *msg;
    use_last_valid_trajectory_ = false;  // 新しいtrajectoryを使用
    
    // デバッグ情報（低頻度）
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Received fresh trajectory with %zu points, backup updated", msg->points.size());
  }
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  
