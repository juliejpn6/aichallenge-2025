#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

// ===========================================
// 道路タイプの列挙型
// ===========================================
enum class RoadType {
  STRAIGHT,      // 直線（曲率 < 0.01）
  GENTLE_CURVE,  // 緩やかなカーブ（曲率 0.01 ~ 0.05）
  SHARP_CURVE    // 急カーブ（曲率 > 0.05）
};

class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;  
  
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;
  
  // ===========================================
  // Pure Pursuit パラメータ（動的変更可能）
  // ===========================================
  double wheel_base_;                  // ホイールベース [m]
  double lookahead_gain_;              // Lookahead距離のゲイン
  double lookahead_min_distance_;      // Lookahead最小距離 [m]
  double speed_proportional_gain_;     // 速度比例ゲイン
  bool use_external_target_vel_;       // 外部目標速度を使用するか
  double external_target_vel_;         // 外部目標速度 [m/s]
  double steering_tire_angle_gain_;    // ステアリングゲイン
  
  // ===========================================
  // 曲率適応制御用のパラメータ（launchファイルで調整可能）
  // ===========================================
  double straight_threshold_;          // 直線判定閾値（デフォルト: 0.01）
  double gentle_curve_threshold_;      // 緩カーブ判定閾値（デフォルト: 0.05）
  double steering_delay_;              // ステアリング遅延時間 [s]（デフォルト: 0.26）
  
  // デバッグ用カウンター
  int log_counter_;
  
 private:
  // ===========================================
  // メイン制御関数
  // ===========================================
  void onTimer();
  bool subscribeMessageAvailable();
  
  // ===========================================
  // 曲率計算関数
  // ===========================================
  
  /**
   * @brief 3点から曲率を計算
   * @param p1 点1
   * @param p2 点2（中心点）
   * @param p3 点3
   * @return 曲率 [1/m]（半径の逆数）
   * 
   * 計算方法：
   * 2つのベクトル (p1→p2) と (p2→p3) の外積から曲率を求める
   * 曲率 = |外積| / (距離1 × 距離2 × 平均距離)
   */
  double calculateCurvature(const TrajectoryPoint& p1, 
                           const TrajectoryPoint& p2, 
                           const TrajectoryPoint& p3);
  
  /**
   * @brief 前方の平均曲率を計算
   * @param current_idx 現在位置のインデックス
   * @return 前方10点の平均曲率 [1/m]
   * 
   * 前方約10m（10点分）の曲率を平均して、
   * 先の道路状況を予測する
   */
  double getAverageCurvatureAhead(size_t current_idx);
  
  /**
   * @brief 曲率から道路タイプを判定
   * @param curvature 曲率 [1/m]
   * @return RoadType (STRAIGHT, GENTLE_CURVE, SHARP_CURVE)
   * 
   * 閾値:
   * - 直線: 曲率 < 0.01
   * - 緩カーブ: 0.01 <= 曲率 < 0.05
   * - 急カーブ: 0.05 <= 曲率
   */
  RoadType classifyRoadType(double curvature);
  
  /**
   * @brief 道路タイプごとの調整係数を取得
   * @param road_type 道路タイプ
   * @param lookahead_factor [出力] Lookahead距離の係数
   * @param speed_limit_factor [出力] 速度制限の係数（現在は無効化）
   * 
   * 係数の意味：
   * - lookahead_factor: 1.0が標準、>1.0で長く、<1.0で短く
   * - speed_limit_factor: 常に1.0（速度制限はトラジェクトリで管理）
   */
  void getControlFactors(RoadType road_type, 
                        double& lookahead_factor, 
                        double& speed_limit_factor);
  
  // ===========================================
  // パラメータ変更時のコールバック
  // ===========================================
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // コールバックハンドル
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
