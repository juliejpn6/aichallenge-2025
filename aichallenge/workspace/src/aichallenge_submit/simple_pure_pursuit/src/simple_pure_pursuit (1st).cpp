// simple_pure_pursuit.cpp - 遅延補償機能追加版（段階1: 0.2秒遅延対策）
// 既存ファイルを置き換えて使用

#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <deque>
#include <cmath>
#include <memory>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

// ===========================
// 遅延補償システム（新規追加）
// ===========================
class DelayCompensationSystem
{
private:
  // 遅延補償パラメータ
  double system_delay_;              // システム遅延時間（秒）
  double max_angular_velocity_;      // 最大角速度制限（rad/s）
  double compensation_gain_;         // 補償ゲイン
  
  // 遅延予測用データ
  std::deque<double> velocity_history_;
  std::deque<double> steering_history_;
  rclcpp::Time last_update_time_;
  bool initialized_;
  
public:
  DelayCompensationSystem() 
    : system_delay_(0.2),           // 0.2秒の固定遅延
      max_angular_velocity_(0.35),  // 角速度制限 0.35rad/s
      compensation_gain_(1.0),
      initialized_(false)
  {
    velocity_history_.resize(10, 0.0);
    steering_history_.resize(5, 0.0);
  }
  
  void setParameters(double system_delay, double max_angular_velocity, double compensation_gain)
  {
    system_delay_ = system_delay;
    max_angular_velocity_ = max_angular_velocity;
    compensation_gain_ = compensation_gain;
  }
  
  // 遅延補償用の先読み距離を計算
  double calculateDelayCompensationDistance(double current_velocity, const rclcpp::Time& current_time)
  {
    // 初期化
    if (!initialized_) {
      last_update_time_ = current_time;
      initialized_ = true;
      return 0.0;
    }
    
    // 速度履歴更新
    velocity_history_.pop_front();
    velocity_history_.push_back(current_velocity);
    
    // 平均速度計算（ノイズ除去）
    double avg_velocity = 0.0;
    for (double v : velocity_history_) {
      avg_velocity += v;
    }
    avg_velocity /= velocity_history_.size();
    
    // 遅延補償距離 = 平均速度 × 遅延時間 × 補償ゲイン
    double delay_distance = avg_velocity * system_delay_ * compensation_gain_;
    
    // 最小・最大制限適用（安全性確保）
    delay_distance = std::clamp(delay_distance, 0.5, 5.0); // 0.5m～5.0m
    
    last_update_time_ = current_time;
    return delay_distance;
  }
  
  // 角速度制限適用（決勝戦仕様）
  double applyAngularVelocityLimit(double target_steering, const rclcpp::Time& current_time)
  {
    if (steering_history_.empty()) {
      steering_history_.push_back(target_steering);
      return target_steering;
    }
    
    // 前回値との差分計算
    double last_steering = steering_history_.back();
    double steering_diff = target_steering - last_steering;
    
    // 時間差分計算
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.001 || dt > 1.0) {
      dt = 0.05; // 20Hz前提のデフォルト値
    }
    
    // 角速度計算
    double angular_velocity = steering_diff / dt;
    
    // 角速度制限適用
    if (std::abs(angular_velocity) > max_angular_velocity_) {
      double limited_change = std::copysign(max_angular_velocity_ * dt, angular_velocity);
      target_steering = last_steering + limited_change;
      
      // デバッグ出力（制限適用時のみ）
      static auto last_limit_log = current_time;
      if ((current_time - last_limit_log).seconds() > 2.0) {
        RCLCPP_INFO(rclcpp::get_logger("delay_compensation"), 
          "角速度制限適用: %.3f → %.3f rad/s", 
          angular_velocity, max_angular_velocity_);
        last_limit_log = current_time;
      }
    }
    
    // 履歴更新
    steering_history_.pop_front();
    steering_history_.push_back(target_steering);
    
    return target_steering;
  }
  
  // デバッグ情報取得
  double getAverageVelocity() const {
    if (velocity_history_.empty()) return 0.0;
    double sum = 0.0;
    for (double v : velocity_history_) sum += v;
    return sum / velocity_history_.size();
  }
  
  double getCurrentCompensationDistance() const {
    return getAverageVelocity() * system_delay_ * compensation_gain_;
  }
};

// 予測平滑化システム（既存を改良）
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
  
  double smoothSteering(double raw_steering, double current_velocity, const rclcpp::Time& current_time)
  {
    // 初期化
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
    
    // 速度適応調整
    double avg_velocity = 0.0;
    for (double v : velocity_history_) {
      avg_velocity += v;
    }
    avg_velocity /= velocity_history_.size();
    
    // 高速時はより保守的に、低速時はより応答的に
    if (avg_velocity > 5.0) {
      double velocity_factor = std::min(1.2, avg_velocity / 10.0);
      smoothed_steering *= (0.8 + 0.2 / velocity_factor);
    }
    
    // 履歴更新
    last_steering_ = smoothed_steering;
    last_time_ = current_time;
    
    return smoothed_steering;
  }
  
  // デバッグ情報取得
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

// 軌道予測システム（遅延補償対応版）
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
  
  // 遅延補償対応の適応的先読み距離計算
  template<typename TrajectoryContainer>
  double calculateAdaptiveLookahead(
    double base_lookahead, 
    double current_velocity,
    const TrajectoryContainer& trajectory,
    size_t closest_idx,
    double delay_compensation_distance = 0.0) const  // 遅延補償距離を追加
  {
    if (closest_idx >= trajectory.size() - 1) {
      return base_lookahead;
    }
    
    // 前方の曲率を分析
    double local_curvature = calculateCurvature(trajectory, closest_idx);
    
    // 曲率に基づく調整係数
    double curvature_factor = 1.0 / (1.0 + std::abs(local_curvature) * curvature_lookahead_gain_);
    
    // 速度に基づく調整
    double velocity_factor = std::sqrt(current_velocity / 10.0); // 基準速度10m/s
    
    // 遅延補償距離を加算（重要な改良点）
    double adjusted_lookahead = (base_lookahead * curvature_factor * velocity_factor) + delay_compensation_distance;
    
    // 30km/h走行時の最適化：8.3m/s前提の調整
    if (current_velocity > 7.0 && current_velocity < 10.0) {
      adjusted_lookahead *= 1.1; // 30km/h付近では若干長めに
    }
    
    return std::clamp(adjusted_lookahead, 3.0, 20.0); // 安全範囲に制限
  }
  
private:
  template<typename TrajectoryContainer>
  double calculateCurvature(const TrajectoryContainer& trajectory, size_t idx) const
  {
    if (idx == 0 || idx >= trajectory.size() - 1) {
      return 0.0;
    }
    
    // 3点による曲率近似
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

// PIMPLパターンの実装クラス（遅延補償対応）
struct SimplePurePursuit::Impl {
  std::unique_ptr<DelayCompensationSystem> delay_compensator;  // 新規追加
  std::unique_ptr<SteeringSmoothing> steering_smoother;
  std::unique_ptr<TrajectoryPredictor> trajectory_predictor;
  
  Impl() {
    delay_compensator = std::make_unique<DelayCompensationSystem>();
    steering_smoother = std::make_unique<SteeringSmoothing>();
    trajectory_predictor = std::make_unique<TrajectoryPredictor>();
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
  pimpl_(std::make_unique<Impl>())
{
  // 既存の予測平滑化パラメータ
  auto smoothing_factor = declare_parameter<float>("steering_smoothing_factor", 0.75);
  auto max_steering_rate = declare_parameter<float>("max_steering_rate", 0.8);
  auto prediction_horizon = declare_parameter<float>("prediction_horizon", 2.5);
  auto curvature_lookahead_gain = declare_parameter<float>("curvature_lookahead_gain", 1.5);
  auto enable_predictive_control = declare_parameter<bool>("enable_predictive_control", true);
  
  // 新規：遅延補償パラメータ
  auto system_delay = declare_parameter<float>("system_delay", 0.2);
  auto max_angular_velocity = declare_parameter<float>("max_angular_velocity", 0.35);
  auto delay_compensation_gain = declare_parameter<float>("delay_compensation_gain", 1.0);
  auto enable_delay_compensation = declare_parameter<bool>("enable_delay_compensation", true);
  
  // システム初期化
  pimpl_->steering_smoother->setParameters(smoothing_factor, max_steering_rate, prediction_horizon);
  pimpl_->trajectory_predictor->setParameters(prediction_horizon, curvature_lookahead_gain);
  pimpl_->delay_compensator->setParameters(system_delay, max_angular_velocity, delay_compensation_gain);
  
  enable_predictive_control_ = enable_predictive_control;
  enable_delay_compensation_ = enable_delay_compensation;
  
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));

  RCLCPP_INFO(get_logger(), "遅延補償Pure Pursuit初期化完了");
  RCLCPP_INFO(get_logger(), "遅延補償: %s, 予測制御: %s", 
              enable_delay_compensation_ ? "有効" : "無効",
              enable_predictive_control_ ? "有効" : "無効");
  RCLCPP_INFO(get_logger(), "角速度制限: %.3f rad/s, システム遅延: %.1f秒", 
              max_angular_velocity, system_delay);
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  cmd.longitudinal.speed = target_longitudinal_vel;
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

  // === 遅延補償対応 lateral control ===
  double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  
  // 遅延補償距離計算（重要な新機能）
  double delay_compensation_distance = 0.0;
  if (enable_delay_compensation_) {
    delay_compensation_distance = pimpl_->delay_compensator->calculateDelayCompensationDistance(
      current_longitudinal_vel, get_clock()->now());
  }
  
  // 予測制御が有効な場合、適応的先読み距離を計算（遅延補償距離込み）
  if (enable_predictive_control_) {
    lookahead_distance = pimpl_->trajectory_predictor->calculateAdaptiveLookahead(
      lookahead_distance, target_longitudinal_vel, trajectory_->points, 
      closet_traj_point_idx, delay_compensation_distance);
  } else {
    // 予測制御無効でも遅延補償は適用
    lookahead_distance += delay_compensation_distance;
  }
  
  // calc center coordinate of rear wheel
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(tf2::getYaw(odometry_->pose.pose.orientation));
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(tf2::getYaw(odometry_->pose.pose.orientation));
  
  // search lookahead point
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });
    
  // 範囲チェック
  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr = trajectory_->points.end() - 1;
  }
  
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  // Debug用lookahead point publish
  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // calc steering angle for lateral control
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                 tf2::getYaw(odometry_->pose.pose.orientation);
  double raw_steering = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

  // === 遅延補償・予測平滑化適用 ===
  double final_steering = raw_steering;
  
  // 1. 予測平滑化（既存機能）
  if (enable_predictive_control_) {
    final_steering = pimpl_->steering_smoother->smoothSteering(
      raw_steering, current_longitudinal_vel, get_clock()->now());
  }
  
  // 2. 角速度制限適用（新規：決勝戦仕様）
  if (enable_delay_compensation_) {
    final_steering = pimpl_->delay_compensator->applyAngularVelocityLimit(
      final_steering, get_clock()->now());
  }
  
  // デバッグ出力（15秒間隔で詳細情報）
  static auto last_debug_time = get_clock()->now();
  if ((get_clock()->now() - last_debug_time).seconds() > 15.0) {
    double vibration_mag = pimpl_->steering_smoother->getVibrationMagnitude();
    double comp_distance = pimpl_->delay_compensator->getCurrentCompensationDistance();
    double avg_velocity = pimpl_->delay_compensator->getAverageVelocity();
    
    RCLCPP_INFO(get_logger(), 
      "=== 遅延補償制御状況 ===");
    RCLCPP_INFO(get_logger(), 
      "速度: %.1f m/s (%.1f km/h), 補償距離: %.2f m", 
      avg_velocity, avg_velocity * 3.6, comp_distance);
    RCLCPP_INFO(get_logger(), 
      "ステアリング: raw=%.3f → smooth=%.3f → final=%.3f", 
      raw_steering, 
      enable_predictive_control_ ? pimpl_->steering_smoother->smoothSteering(raw_steering, current_longitudinal_vel, get_clock()->now()) : raw_steering,
      final_steering);
    RCLCPP_INFO(get_logger(), 
      "先読み距離: base=%.2f + delay=%.2f = total=%.2f m", 
      lookahead_distance - delay_compensation_distance, delay_compensation_distance, lookahead_distance);
    RCLCPP_INFO(get_logger(), 
      "振動抑制度: %.4f", vibration_mag);
    
    last_debug_time = get_clock()->now();
  }

  cmd.lateral.steering_tire_angle = steering_tire_angle_gain_ * final_steering;

  // Publish commands
  pub_cmd_->publish(cmd);
  
  // Raw command (for debugging)
  AckermannControlCommand raw_cmd = cmd;
  raw_cmd.lateral.steering_tire_angle = raw_steering * steering_tire_angle_gain_;
  pub_raw_cmd_->publish(raw_cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}

}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
