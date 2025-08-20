// simple_pure_pursuit.cpp - 蛇行抑制機能を追加した改良版
// カーブ後の直線での滑らかなハンドル戻し機能を実装

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

// 軌道状態分析クラス
class TrajectoryAnalyzer
{
private:
  std::deque<double> curvature_history_;
  std::deque<double> lateral_error_history_;
  size_t history_size_;
  double curve_threshold_;
  double straight_threshold_;
  bool in_curve_;
  int curve_exit_counter_;
  
public:
  TrajectoryAnalyzer() 
    : history_size_(10),
      curve_threshold_(0.05),
      straight_threshold_(0.02),
      in_curve_(false),
      curve_exit_counter_(0)
  {
    curvature_history_.resize(history_size_, 0.0);
    lateral_error_history_.resize(5, 0.0);
  }
  
  struct TrajectoryState {
    bool is_in_curve;
    bool just_exited_curve;
    double avg_curvature;
    double lateral_error_trend;
    bool is_oscillating;
  };
  
  TrajectoryState analyzeTrajectory(double current_curvature, double lateral_error)
  {
    // 履歴更新
    curvature_history_.pop_front();
    curvature_history_.push_back(std::abs(current_curvature));
    
    lateral_error_history_.pop_front();
    lateral_error_history_.push_back(lateral_error);
    
    // 平均曲率計算
    double avg_curvature = 0.0;
    for (double c : curvature_history_) {
      avg_curvature += c;
    }
    avg_curvature /= curvature_history_.size();
    
    // カーブ状態判定
    bool was_in_curve = in_curve_;
    in_curve_ = avg_curvature > curve_threshold_;
    
    // カーブ出口検出
    bool just_exited = false;
    if (was_in_curve && !in_curve_) {
      curve_exit_counter_ = 8; // 8サイクル間カーブ出口状態を維持
      just_exited = true;
    } else if (curve_exit_counter_ > 0) {
      curve_exit_counter_--;
      just_exited = true;
    }
    
    // 横偏差の振動検出
    bool is_oscillating = detectLateralOscillation();
    
    // 横偏差トレンド
    double lateral_trend = calculateLateralTrend();
    
    TrajectoryState state;
    state.is_in_curve = in_curve_;
    state.just_exited_curve = just_exited;
    state.avg_curvature = avg_curvature;
    state.lateral_error_trend = lateral_trend;
    state.is_oscillating = is_oscillating;
    
    return state;
  }
  
private:
  bool detectLateralOscillation() const
  {
    if (lateral_error_history_.size() < 4) return false;
    
    int sign_changes = 0;
    for (size_t i = 1; i < lateral_error_history_.size(); ++i) {
      if ((lateral_error_history_[i-1] > 0) != (lateral_error_history_[i] > 0)) {
        sign_changes++;
      }
    }
    
    // 2回以上の符号変化で振動と判定
    return sign_changes >= 2;
  }
  
  double calculateLateralTrend() const
  {
    if (lateral_error_history_.size() < 3) return 0.0;
    
    // 最近3点の傾向
    double trend = 0.0;
    for (size_t i = 2; i < lateral_error_history_.size(); ++i) {
      trend += lateral_error_history_[i] - lateral_error_history_[i-2];
    }
    
    return trend / (lateral_error_history_.size() - 2);
  }
};

// 予測平滑化用のクラス（蛇行抑制機能強化）
class SteeringSmoothing
{
private:
  std::deque<double> steering_history_;
  std::deque<double> velocity_history_;
  double last_steering_;
  double target_steering_; // 目標ステアリング角
  rclcpp::Time last_time_;
  bool initialized_;
  
  // パラメータ
  double smoothing_factor_;
  double max_steering_rate_;
  double prediction_horizon_;
  size_t history_size_;
  
  // 蛇行抑制用パラメータ
  double return_to_center_gain_;
  double oscillation_damping_factor_;
  
public:
  SteeringSmoothing() 
    : last_steering_(0.0),
      target_steering_(0.0),
      initialized_(false),
      smoothing_factor_(0.75),
      max_steering_rate_(0.8),
      prediction_horizon_(2.0),
      history_size_(5),
      return_to_center_gain_(0.3),
      oscillation_damping_factor_(0.6)
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
  
  double smoothSteering(
    double raw_steering, 
    double current_velocity, 
    const rclcpp::Time& current_time,
    const TrajectoryAnalyzer::TrajectoryState& traj_state)
  {
    // 初期化
    if (!initialized_) {
      last_steering_ = raw_steering;
      target_steering_ = raw_steering;
      last_time_ = current_time;
      initialized_ = true;
      return raw_steering;
    }
    
    // 1. 履歴更新
    steering_history_.pop_front();
    steering_history_.push_back(raw_steering);
    
    velocity_history_.pop_front();
    velocity_history_.push_back(current_velocity);
    
    // 2. カーブ出口での特別処理
    double smoothed_steering = raw_steering;
    
    if (traj_state.just_exited_curve) {
      // カーブ出口: 徐々にセンターに戻す
      target_steering_ = raw_steering * (1.0 - return_to_center_gain_);
      smoothed_steering = interpolateToTarget(raw_steering, target_steering_, 0.7);
    }
    else if (traj_state.is_oscillating && !traj_state.is_in_curve) {
      // 直線での振動検出: 強い減衰
      smoothed_steering = applyOscillationDamping(raw_steering);
    }
    else {
      // 通常の指数移動平均
      smoothed_steering = applyExponentialSmoothing(raw_steering);
    }
    
    // 3. 角速度制限適用
    double dt = (current_time - last_time_).seconds();
    if (dt > 0.001 && dt < 1.0) {
      smoothed_steering = applySteeringRateLimit(smoothed_steering, dt, traj_state);
    }
    
    // 4. 速度適応調整
    smoothed_steering = applyVelocityAdaptation(smoothed_steering, current_velocity);
    
    // 5. 履歴更新
    last_steering_ = smoothed_steering;
    last_time_ = current_time;
    
    return smoothed_steering;
  }
  
private:
  double applyExponentialSmoothing(double raw_steering)
  {
    double smoothed = raw_steering;
    for (int i = steering_history_.size() - 2; i >= 0; --i) {
      smoothed = smoothing_factor_ * smoothed + 
                (1.0 - smoothing_factor_) * steering_history_[i];
    }
    return smoothed;
  }
  
  double applyOscillationDamping(double raw_steering)
  {
    // 振動時は過去の値により強く依存
    double damped_factor = smoothing_factor_ * oscillation_damping_factor_;
    return damped_factor * raw_steering + (1.0 - damped_factor) * last_steering_;
  }
  
  double interpolateToTarget(double current, double target, double blend_ratio)
  {
    return blend_ratio * current + (1.0 - blend_ratio) * target;
  }
  
  double applySteeringRateLimit(double target_steering, double dt, 
                               const TrajectoryAnalyzer::TrajectoryState& traj_state)
  {
    double steering_rate = (target_steering - last_steering_) / dt;
    
    // カーブ出口では更に保守的な制限
    double rate_limit = max_steering_rate_;
    if (traj_state.just_exited_curve) {
      rate_limit *= 0.7; // 30%減速
    }
    
    if (std::abs(steering_rate) > rate_limit) {
      double limited_change = std::copysign(rate_limit * dt, steering_rate);
      return last_steering_ + limited_change;
    }
    
    return target_steering;
  }
  
  double applyVelocityAdaptation(double steering, double velocity)
  {
    double avg_velocity = 0.0;
    for (double v : velocity_history_) {
      avg_velocity += v;
    }
    avg_velocity /= velocity_history_.size();
    
    // 高速時はより保守的に
    if (avg_velocity > 8.0) {
      double velocity_factor = std::min(1.3, avg_velocity / 12.0);
      steering *= (0.75 + 0.25 / velocity_factor);
    }
    
    return steering;
  }
  
public:
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
    size_t closest_idx,
    const TrajectoryAnalyzer::TrajectoryState& traj_state) const
  {
    if (closest_idx >= trajectory.size() - 1) {
      return base_lookahead;
    }
    
    // 前方の曲率を分析
    double local_curvature = calculateCurvature(trajectory, closest_idx);
    
    // 曲率に基づく調整係数
    double curvature_factor = 1.0 / (1.0 + std::abs(local_curvature) * curvature_lookahead_gain_);
    
    // カーブ出口での特別調整
    if (traj_state.just_exited_curve) {
      curvature_factor *= 1.2; // 先読み距離を20%増加
    }
    
    // 振動検出時の調整
    if (traj_state.is_oscillating && !traj_state.is_in_curve) {
      curvature_factor *= 1.3; // さらに先読み距離を増加
    }
    
    // 速度に基づく調整
    double velocity_factor = std::sqrt(current_velocity / 10.0);
    
    return base_lookahead * curvature_factor * velocity_factor;
  }
  
  // 軌道の曲率を計算
  template<typename TrajectoryContainer>
  double calculateTrajectoryeCurvature(const TrajectoryContainer& trajectory, size_t idx) const
  {
    return calculateCurvature(trajectory, idx);
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

// PIMPLパターンの実装クラス（拡張版）
struct SimplePurePursuit::Impl {
  std::unique_ptr<SteeringSmoothing> steering_smoother;
  std::unique_ptr<TrajectoryPredictor> trajectory_predictor;
  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer;
  
  Impl() {
    steering_smoother = std::make_unique<SteeringSmoothing>();
    trajectory_predictor = std::make_unique<TrajectoryPredictor>();
    trajectory_analyzer = std::make_unique<TrajectoryAnalyzer>();
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
  // 予測平滑化パラメータ
  auto smoothing_factor = declare_parameter<float>("steering_smoothing_factor", 0.85);
  auto max_steering_rate = declare_parameter<float>("max_steering_rate", 1.0);
  auto prediction_horizon = declare_parameter<float>("prediction_horizon", 4.0);
  auto curvature_lookahead_gain = declare_parameter<float>("curvature_lookahead_gain", 1.3);
  auto enable_predictive_control = declare_parameter<bool>("enable_predictive_control", true);
  
  // 循環軌道パラメータ（既存）
  auto enable_circular_trajectory = declare_parameter<bool>("enable_circular_trajectory", true);
  auto circular_search_distance_ratio = declare_parameter<float>("circular_search_distance_ratio", 0.8);
  
  // PIMPLパターンを通じた初期化
  pimpl_->steering_smoother->setParameters(smoothing_factor, max_steering_rate, prediction_horizon);
  pimpl_->trajectory_predictor->setParameters(prediction_horizon, curvature_lookahead_gain);
  
  enable_predictive_control_ = enable_predictive_control;
  
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

  RCLCPP_INFO(get_logger(), "Enhanced Pure Pursuit initialized - Anti-Weaving Control: %s", 
              enable_predictive_control_ ? "ENABLED" : "DISABLED");
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

  // === 改良されたlateral control ===
  double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  
  // 軌道状態分析
  double current_curvature = 0.0;
  double lateral_error = 0.0;
  TrajectoryAnalyzer::TrajectoryState traj_state;
  
  if (enable_predictive_control_) {
    // 現在の曲率と横偏差を計算
    current_curvature = pimpl_->trajectory_predictor->calculateTrajectoryeCurvature(
      trajectory_->points, closet_traj_point_idx);
    lateral_error = calcLateralDeviation(closet_traj_point.pose, odometry_->pose.pose.position);
    
    // 軌道状態分析
    traj_state = pimpl_->trajectory_analyzer->analyzeTrajectory(current_curvature, lateral_error);
    
    // 適応的先読み距離計算
    lookahead_distance = pimpl_->trajectory_predictor->calculateAdaptiveLookahead(
      lookahead_distance, target_longitudinal_vel, trajectory_->points, 
      closet_traj_point_idx, traj_state);
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

  // === 蛇行抑制予測平滑化適用 ===
  double final_steering = raw_steering;
  if (enable_predictive_control_) {
    final_steering = pimpl_->steering_smoother->smoothSteering(
      raw_steering, current_longitudinal_vel, get_clock()->now(), traj_state);
      
    // デバッグ出力（5秒間隔）
    static auto last_debug_time = get_clock()->now();
    if ((get_clock()->now() - last_debug_time).seconds() > 5.0) {
      double vibration_mag = pimpl_->steering_smoother->getVibrationMagnitude();
      RCLCPP_INFO(get_logger(), 
        "Control State: curve=%s, exit=%s, osc=%s | Steering: raw=%.3f→smooth=%.3f | Vib=%.4f",
        traj_state.is_in_curve ? "Y" : "N",
        traj_state.just_exited_curve ? "Y" : "N", 
        traj_state.is_oscillating ? "Y" : "N",
        raw_steering, final_steering, vibration_mag);
      last_debug_time = get_clock()->now();
    }
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
