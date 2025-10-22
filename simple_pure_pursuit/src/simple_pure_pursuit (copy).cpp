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
  // パラメータ初期化
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 3.7)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.5)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  // 曲率適応制御パラメータ（launchファイルで調整可能）
  straight_threshold_(declare_parameter<float>("straight_threshold", 0.01)),
  gentle_curve_threshold_(declare_parameter<float>("gentle_curve_threshold", 0.05)),
  steering_delay_(declare_parameter<float>("steering_delay", 0.26)),  // 260ms
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
  
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "SimplePurePursuit 起動（曲率適応型）");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "  external_target_vel: %.2f m/s", external_target_vel_);
  RCLCPP_INFO(this->get_logger(), "  lookahead_gain: %.2f", lookahead_gain_);
  RCLCPP_INFO(this->get_logger(), "  lookahead_min_distance: %.2f m", lookahead_min_distance_);
  RCLCPP_INFO(this->get_logger(), "  steering_delay: %.3f s", steering_delay_);
  RCLCPP_INFO(this->get_logger(), "  straight_threshold: %.4f", straight_threshold_);
  RCLCPP_INFO(this->get_logger(), "  gentle_curve_threshold: %.4f", gentle_curve_threshold_);
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
    else if (param.get_name() == "lookahead_gain") {
      lookahead_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "✅ lookahead_gain updated to %.2f", lookahead_gain_);
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
 * 
 * アルゴリズム：
 * 1. 2つのベクトルを計算: v1 = p2 - p1, v2 = p3 - p2
 * 2. 外積を計算: cross = v1 × v2
 * 3. 各ベクトルの長さを計算
 * 4. 曲率 = |cross| / (|v1| × |v2| × (|v1| + |v2|) / 2)
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
  // これは円の半径の逆数を近似的に求める式
  double curvature = std::fabs(cross) / (d1 * d2 * (d1 + d2) / 2.0);
  
  return curvature;
}

/**
 * @brief 前方の平均曲率を計算
 * 
 * 前方10点（約10m）の曲率を平均して、
 * これから進む経路の曲率を予測する
 */
double SimplePurePursuit::getAverageCurvatureAhead(size_t current_idx)
{
  // データ不足チェック
  if (current_idx + 12 >= trajectory_->points.size()) {
    return 0.0;
  }
  
  double sum_curvature = 0.0;
  int count = 0;
  
  // 前方10点の曲率を計算
  for (size_t i = current_idx; i < current_idx + 20; i++) {
    if (i + 2 < trajectory_->points.size()) {
      double curvature = calculateCurvature(
        trajectory_->points[i],
        trajectory_->points[i + 1],
        trajectory_->points[i + 2]
      );
      sum_curvature += curvature;
      count++;
    }
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
 * 
 * ★速度制限は無効化（トラジェクトリファイルで管理）
 * Lookahead距離のみ調整
 */
void SimplePurePursuit::getControlFactors(
  RoadType road_type, 
  double& lookahead_factor, 
  double& speed_limit_factor)
{
  switch (road_type) {
    case RoadType::STRAIGHT:
      // 直線：Lookaheadを20%延長
      lookahead_factor = 1.2;
      speed_limit_factor = 1.0;  // 速度制限なし
      break;
      
    case RoadType::GENTLE_CURVE:
      // 緩やかなカーブ：標準設定
      lookahead_factor = 1.1;
      speed_limit_factor = 1.0;  // ★速度制限を無効化（0.9 → 1.0）
      break;
      
    case RoadType::SHARP_CURVE:
      // 急カーブ：Lookaheadを20%短縮
      lookahead_factor = 1.0;
      speed_limit_factor = 1.0;  // ★速度制限を無効化（0.75 → 1.0）
      break;
  }
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
  // 【新機能】曲率計算と道路タイプ判定
  // ===========================================
  
  // 前方の平均曲率を計算
  double avg_curvature = getAverageCurvatureAhead(closet_traj_point_idx);
  
  // 道路タイプを判定
  RoadType road_type = classifyRoadType(avg_curvature);
  
  // 道路タイプごとの調整係数を取得
  double lookahead_factor = 1.0;
  double speed_limit_factor = 1.0;
  getControlFactors(road_type, lookahead_factor, speed_limit_factor);
  
  // ===========================================
  // 縦方向制御（速度・加速度）
  // ===========================================
  
  // 目標速度の取得（外部速度 or 軌道速度）
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  
  // ★速度制限を無効化（トラジェクトリファイルで速度管理）
  // target_longitudinal_vel *= speed_limit_factor;  // コメントアウト
  
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
  
  // 【新機能】Lookahead距離の計算（曲率適応）
  double base_lookahead = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  double lookahead_distance = base_lookahead * lookahead_factor;
  
  // 後輪中心座標の計算
  double current_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(current_yaw);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(current_yaw);
  
  // ===========================================
  // 【新機能】ステアリング遅延補償
  // ===========================================
  
  // 遅延時間分だけ先の位置を予測
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
    
    RCLCPP_INFO(this->get_logger(), 
                "[制御] 道路: %s | 曲率: %.4f | Lookahead: %.2fm | "
                "目標速度: %.2fm/s | 実速度: %.2fm/s",
                road_type_str.c_str(), 
                avg_curvature, 
                lookahead_distance, 
                target_longitudinal_vel,
                current_longitudinal_vel);
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
