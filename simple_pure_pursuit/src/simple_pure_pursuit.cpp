#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

// ===========================================
// コンストラクタ
// ===========================================
SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // 既存パラメータ
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  
  // ★★★ 変更：lookahead_gain → lookahead_time ★★★
  lookahead_time_(declare_parameter<float>("lookahead_time", 2.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 2.0)),
  
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  // 曲率適応型パラメータ
  straight_threshold_(declare_parameter<float>("straight_threshold", 0.012)),
  gentle_curve_threshold_(declare_parameter<float>("gentle_curve_threshold", 0.08)),
  steering_delay_(declare_parameter<float>("steering_delay", 0.30)),
  // 平滑化パラメータ
  lookahead_factor_change_rate_(declare_parameter<float>("lookahead_factor_change_rate", 0.3)),
  previous_lookahead_factor_(1.0),  // 初期値は1.0（標準）
  // ステアリング状態考慮パラメータ
  cornering_threshold_(declare_parameter<float>("cornering_threshold", 0.05)),  // 約3度
  previous_steering_angle_(0.0),
  corner_exit_lookahead_boost_(declare_parameter<float>("corner_exit_lookahead_boost", 1.0)),
  // 道路タイプ別調整係数パラメータ
  // 【直線用パラメータ】
  straight_lookahead_factor_(declare_parameter<float>("straight_lookahead_factor", 1.0)),
  straight_speed_limit_factor_(declare_parameter<float>("straight_speed_limit_factor", 1.0)),
  // 【緩やかなカーブ用パラメータ】
  gentle_curve_lookahead_factor_(declare_parameter<float>("gentle_curve_lookahead_factor", 1.0)),
  gentle_curve_speed_limit_factor_(declare_parameter<float>("gentle_curve_speed_limit_factor", 0.7)),
  // 【急カーブ用パラメータ】
  sharp_curve_lookahead_factor_(declare_parameter<float>("sharp_curve_lookahead_factor", 1.0)),
  sharp_curve_speed_limit_factor_(declare_parameter<float>("sharp_curve_speed_limit_factor", 0.3)),
  // デバッグ用
  log_counter_(0)
{
  // パブリッシャー作成
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  // サブスクライバー作成
  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  // パラメータ変更コールバック設定
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&SimplePurePursuit::onParameter, this, std::placeholders::_1));
  
  // ========================================
  // 起動時のパラメータ表示
  // ========================================
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "SimplePurePursuit 起動（時間ベースLookahead版）");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "--- 基本パラメータ ---");
  RCLCPP_INFO(this->get_logger(), "  external_target_vel: %.2f m/s", external_target_vel_);
  
  // ★★★ 変更：lookahead_gain → lookahead_time ★★★
  RCLCPP_INFO(this->get_logger(), "  lookahead_time: %.2f 秒", lookahead_time_);
  RCLCPP_INFO(this->get_logger(), "  lookahead_min_distance: %.2f m", lookahead_min_distance_);
  
  RCLCPP_INFO(this->get_logger(), "  steering_delay: %.3f s", steering_delay_);
  RCLCPP_INFO(this->get_logger(), "  straight_threshold: %.4f", straight_threshold_);
  RCLCPP_INFO(this->get_logger(), "  gentle_curve_threshold: %.4f", gentle_curve_threshold_);
  RCLCPP_INFO(this->get_logger(), "--- 平滑化パラメータ ---");
  RCLCPP_INFO(this->get_logger(), "  lookahead_factor_change_rate: %.2f", lookahead_factor_change_rate_);
  RCLCPP_INFO(this->get_logger(), "--- ステアリング状態考慮 ---");
  RCLCPP_INFO(this->get_logger(), "  cornering_threshold: %.3f rad (%.1f deg)", 
              cornering_threshold_, cornering_threshold_ * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "  corner_exit_lookahead_boost: %.2f", corner_exit_lookahead_boost_);
  
  // 道路タイプ別調整係数の表示
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "--- 道路タイプ別調整係数 ---");
  RCLCPP_INFO(this->get_logger(), "【直線】");
  RCLCPP_INFO(this->get_logger(), "  lookahead_factor: %.2f", straight_lookahead_factor_);
  RCLCPP_INFO(this->get_logger(), "  speed_limit_factor: %.2f (%.0f%%)", 
              straight_speed_limit_factor_, straight_speed_limit_factor_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "【緩やかなカーブ】");
  RCLCPP_INFO(this->get_logger(), "  lookahead_factor: %.2f", gentle_curve_lookahead_factor_);
  RCLCPP_INFO(this->get_logger(), "  speed_limit_factor: %.2f (%.0f%%)", 
              gentle_curve_speed_limit_factor_, gentle_curve_speed_limit_factor_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "【急カーブ】");
  RCLCPP_INFO(this->get_logger(), "  lookahead_factor: %.2f", sharp_curve_lookahead_factor_);
  RCLCPP_INFO(this->get_logger(), "  speed_limit_factor: %.2f (%.0f%%)", 
              sharp_curve_speed_limit_factor_, sharp_curve_speed_limit_factor_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "========================================");
  
  // タイマー起動（100Hz = 10ms）
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

// ===========================================
// パラメータ変更時のコールバック
// ===========================================
rcl_interfaces::msg::SetParametersResult SimplePurePursuit::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "external_target_vel") {
      external_target_vel_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ external_target_vel updated to %.2f m/s", external_target_vel_);
    }
    else if (param.get_name() == "use_external_target_vel") {
      use_external_target_vel_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "✅ use_external_target_vel updated to %s", 
                  use_external_target_vel_ ? "true" : "false");
    }
    // ★★★ 変更：lookahead_gain → lookahead_time ★★★
    else if (param.get_name() == "lookahead_time") {
      lookahead_time_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ lookahead_time updated to %.2f s", lookahead_time_);
    }
    else if (param.get_name() == "lookahead_min_distance") {
      lookahead_min_distance_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ lookahead_min_distance updated to %.2f m", lookahead_min_distance_);
    }
    else if (param.get_name() == "speed_proportional_gain") {
      speed_proportional_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ speed_proportional_gain updated to %.2f", speed_proportional_gain_);
    }
    else if (param.get_name() == "steering_tire_angle_gain") {
      steering_tire_angle_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ steering_tire_angle_gain updated to %.2f", steering_tire_angle_gain_);
    }
    else if (param.get_name() == "lookahead_factor_change_rate") {
      lookahead_factor_change_rate_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ lookahead_factor_change_rate updated to %.2f", lookahead_factor_change_rate_);
    }
    else if (param.get_name() == "cornering_threshold") {
      cornering_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ cornering_threshold updated to %.3f rad", cornering_threshold_);
    }
    else if (param.get_name() == "corner_exit_lookahead_boost") {
      corner_exit_lookahead_boost_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ corner_exit_lookahead_boost updated to %.2f", corner_exit_lookahead_boost_);
    }
  }
  
  return result;
}

// ===========================================
// ゼロコマンド生成
// ===========================================
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

// ===========================================
// 曲率計算関数の実装
// ===========================================

/**
 * @brief 3点から曲率を計算
 */
double SimplePurePursuit::calculateCurvature(
  const TrajectoryPoint& p1, 
  const TrajectoryPoint& p2, 
  const TrajectoryPoint& p3)
{
  // ベクトル計算
  double dx1 = p2.pose.position.x - p1.pose.position.x;
  double dy1 = p2.pose.position.y - p1.pose.position.y;
  double dx2 = p3.pose.position.x - p2.pose.position.x;
  double dy2 = p3.pose.position.y - p2.pose.position.y;
  
  // 外積（z成分のみ、2D平面）
  double cross = dx1 * dy2 - dy1 * dx2;
  
  // 各ベクトルの長さ
  double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
  
  // ゼロ除算防止
  if (d1 < 0.01 || d2 < 0.01) {
    return 0.0;
  }
  
  // 曲率 = |外積| / (距離の積 × 平均距離)
  double curvature = std::fabs(cross) / (d1 * d2 * (d1 + d2) / 2.0);
  
  return curvature;
}

/**
 * @brief 前方の平均曲率を計算
 */
double SimplePurePursuit::getAverageCurvatureAhead(size_t current_idx)
{
  // データ不足チェック
  if (current_idx + 22 >= trajectory_->points.size()) {
    return 0.0;
  }
  
  double sum_curvature = 0.0;
  int count = 0;
  
  // 前方20点の曲率を計算
  for (size_t i = current_idx; i < current_idx + 30 && i + 2 < trajectory_->points.size(); i++) {
    double curvature = calculateCurvature(
      trajectory_->points[i],
      trajectory_->points[i + 1],
      trajectory_->points[i + 2]
    );
    sum_curvature += curvature;
    count++;
  }
  
  // 平均を返す
  return (count > 0) ? (sum_curvature / count) : 0.0;
}

/**
 * @brief 曲率から道路タイプを判定
 */
RoadType SimplePurePursuit::classifyRoadType(double curvature)
{
  if (curvature < straight_threshold_) {
    return RoadType::STRAIGHT;
  } else if (curvature < gentle_curve_threshold_) {
    return RoadType::GENTLE_CURVE;
  } else {
    return RoadType::SHARP_CURVE;
  }
}

/**
 * @brief 道路タイプごとの調整係数を取得
 */
void SimplePurePursuit::getControlFactors(
  RoadType road_type, 
  double& lookahead_factor, 
  double& speed_limit_factor)
{
  switch (road_type) {
    case RoadType::STRAIGHT:
      // 直線：パラメータで設定された値を使用
      lookahead_factor = straight_lookahead_factor_;
      speed_limit_factor = straight_speed_limit_factor_;
      break;
      
    case RoadType::GENTLE_CURVE:
      // 緩やかなカーブ：パラメータで設定された値を使用
      lookahead_factor = gentle_curve_lookahead_factor_;
      speed_limit_factor = gentle_curve_speed_limit_factor_;
      break;
      
    case RoadType::SHARP_CURVE:
      // 急カーブ：パラメータで設定された値を使用
      lookahead_factor = sharp_curve_lookahead_factor_;
      speed_limit_factor = sharp_curve_speed_limit_factor_;
      break;
  }
}

// ===========================================
// 平滑化関数の実装
// ===========================================

/**
 * @brief Lookahead Factorを平滑化する
 */
double SimplePurePursuit::smoothLookaheadFactor(double target_factor)
{
  // 前回値から目標値へ、設定された変化率で徐々に近づける
  double smoothed_factor = previous_lookahead_factor_ + 
                          (target_factor - previous_lookahead_factor_) * lookahead_factor_change_rate_;
  
  // 前回値を更新（次回のために保存）
  previous_lookahead_factor_ = smoothed_factor;
  
  return smoothed_factor;
}

// ===========================================
// ステアリング状態考慮関数の実装
// ===========================================

/**
 * @brief ステアリング状態を考慮してLookahead Factorを調整
 */
double SimplePurePursuit::adjustFactorForCorneringState(
    double base_factor, 
    double current_steering,
    RoadType road_type)
{
  // ステアリング角度の絶対値を取得
  double abs_steering = std::abs(current_steering);
  
  // コーナー走行中の判定
  bool is_cornering = (abs_steering > cornering_threshold_);
  
  // コーナー出口の特別処理
  if (is_cornering && road_type == RoadType::STRAIGHT) {
    // コーナー出口と判定
    double exit_factor = base_factor * corner_exit_lookahead_boost_;
    
    // ログ出力（デバッグ用、50回に1回）
    if (log_counter_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "🔄 コーナー出口検出: factor: %.2f → %.2f (boost: %.2f, steering: %.1f deg)",
                  base_factor, exit_factor, corner_exit_lookahead_boost_,
                  abs_steering * 180.0 / M_PI);
    }
    
    return exit_factor;
  }
  
  // 急カーブ走行中は、factorをやや抑える（安定性重視）
  if (is_cornering && road_type == RoadType::SHARP_CURVE) {
    return base_factor * 0.95;  // 5%削減
  }
  
  // その他の場合は、base_factorをそのまま使用
  return base_factor;
}

// ===========================================
// メイン制御ループ
// ===========================================
void SimplePurePursuit::onTimer()
{
  // データ受信チェック
  if (!subscribeMessageAvailable()) {
    return;
  }

  // 最近傍点のインデックスを取得
  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // ゼロコマンド初期化
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // 最近傍の軌道点を取得
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // ===========================================
  // 曲率計算と道路タイプ判定
  // ===========================================
  
  // 前方の平均曲率を計算
  double avg_curvature = getAverageCurvatureAhead(closet_traj_point_idx);
  
  // 道路タイプを判定
  RoadType road_type = classifyRoadType(avg_curvature);
  
  // 道路タイプごとの調整係数を取得
  double target_lookahead_factor = 1.0;
  double speed_limit_factor = 1.0;
  getControlFactors(road_type, target_lookahead_factor, speed_limit_factor);
  
  // 平滑化を適用
  double smoothed_factor = smoothLookaheadFactor(target_lookahead_factor);
  
  // ステアリング状態を考慮
  double final_lookahead_factor = adjustFactorForCorneringState(
    smoothed_factor, 
    previous_steering_angle_,
    road_type
  );
  
  // ===========================================
  // 縦方向制御（速度・加速度）
  // ===========================================
  
  // 目標速度の取得（外部速度 or 軌道速度）
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  
  // 現在速度
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  // 速度コマンド設定
  cmd.longitudinal.speed = target_longitudinal_vel;
  
  // 加速度計算（P制御）
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

  // ===========================================
  // 横方向制御（ステアリング）
  // ===========================================
  
  // ★★★ 変更：時間ベースのLookahead距離計算 ★★★
  // 計算式: max(速度 × 先読み時間, 最小距離)
  double time_based_lookahead = target_longitudinal_vel * lookahead_time_;
  double base_lookahead = std::max(time_based_lookahead, lookahead_min_distance_);
  double lookahead_distance = base_lookahead * final_lookahead_factor;
  
  // 後輪中心座標の計算
  double current_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(current_yaw);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(current_yaw);
  
  // ステアリング遅延補償
  double predicted_x = rear_x + current_longitudinal_vel * std::cos(current_yaw) * steering_delay_;
  double predicted_y = rear_y + current_longitudinal_vel * std::sin(current_yaw) * steering_delay_;
  
  // 予測位置からLookahead点を探索
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closet_traj_point_idx, 
    trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - predicted_x, 
                       point.pose.position.y - predicted_y) >= lookahead_distance;
    });
  
  // Lookahead点が見つからない場合は最後の点を使用
  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr = trajectory_->points.end() - 1;
  }
  
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  // Lookahead点をパブリッシュ（可視化用）
  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // ステアリング角度の計算（Pure Pursuitアルゴリズム）
  double alpha = std::atan2(lookahead_point_y - predicted_y, lookahead_point_x - predicted_x) - current_yaw;
  cmd.lateral.steering_tire_angle =
    steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

  // 今回のステアリング角度を保存（次回の判定用）
  previous_steering_angle_ = cmd.lateral.steering_tire_angle;

  // コマンドをパブリッシュ
  pub_cmd_->publish(cmd);
  
  // Raw コマンド（ゲイン適用前）もパブリッシュ
  cmd.lateral.steering_tire_angle /= steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
  
  // ===========================================
  // デバッグログ出力（50回に1回 = 0.5秒ごと）
  // ===========================================
  if (log_counter_++ % 50 == 0) {
    std::string road_type_str = 
      (road_type == RoadType::STRAIGHT) ? "直線" :
      (road_type == RoadType::GENTLE_CURVE) ? "緩カーブ" : "急カーブ";
    
    // ★★★ 変更：先読み時間を計算して表示 ★★★
    double actual_lookahead_time = (current_longitudinal_vel > 0.1) ? 
      (lookahead_distance / current_longitudinal_vel) : 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
                "[制御] 道路: %s | 曲率: %.4f | Factor: %.2f→%.2f→%.2f | Lookahead: %.2fm (%.2fs先) | ステア: %.2f°",
                road_type_str.c_str(), 
                avg_curvature, 
                target_lookahead_factor,
                smoothed_factor,
                final_lookahead_factor,
                lookahead_distance,
                actual_lookahead_time,
                previous_steering_angle_ * 180.0 / M_PI);
  }
}

// ===========================================
// データ受信チェック
// ===========================================
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
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory points is empty");
    return false;
  }
  return true;
}

}  // namespace simple_pure_pursuit

// ===========================================
// メイン関数
// ===========================================
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
