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
// 道路タイプの定義
// ===========================================
enum class RoadType {
  STRAIGHT,       // 直線
  GENTLE_CURVE,   // 緩やかなカーブ
  SHARP_CURVE     // 急カーブ
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
  // 基本パラメータ
  // ===========================================
  double wheel_base_;
  
  // ★★★ 変更:lookahead_gain → lookahead_time ★★★
  double lookahead_time_;              // 先読み時間(秒)
  double lookahead_min_distance_;      // 最小Lookahead距離(m)
  
  double speed_proportional_gain_;
  bool use_external_target_vel_;
  double external_target_vel_;
  double steering_tire_angle_gain_;
  
  // ===========================================
  // 曲率適応型パラメータ
  // ===========================================
  double straight_threshold_;        // 直線判定の閾値
  double gentle_curve_threshold_;    // 緩カーブ判定の閾値
  double steering_delay_;            // ステアリング遅延時間
  
  // ===========================================
  // 平滑化パラメータ
  // ===========================================
  double lookahead_factor_change_rate_;  // Lookahead Factor変化率
  double previous_lookahead_factor_;     // 前回のLookahead Factor
  
  // ===========================================
  // ステアリング状態考慮パラメータ
  // ===========================================
  double cornering_threshold_;           // コーナー走行中判定の閾値
  double previous_steering_angle_;       // 前回のステアリング角度
  double corner_exit_lookahead_boost_;   // コーナー出口でのLookahead延長率
  
  // ===========================================
  // 道路タイプ別調整係数パラメータ
  // ===========================================
  
  // 【直線用パラメータ】
  double straight_lookahead_factor_;    // 直線でのLookahead調整係数
  double straight_speed_limit_factor_;  // 直線での速度制限係数
  
  // 【緩やかなカーブ用パラメータ】
  double gentle_curve_lookahead_factor_;    // 緩カーブでのLookahead調整係数
  double gentle_curve_speed_limit_factor_;  // 緩カーブでの速度制限係数
  
  // 【急カーブ用パラメータ】
  double sharp_curve_lookahead_factor_;     // 急カーブでのLookahead調整係数
  double sharp_curve_speed_limit_factor_;   // 急カーブでの速度制限係数
  
  // ===========================================
  // デバッグ用
  // ===========================================
  int log_counter_;  // ログ出力カウンター
  
 private:
  // ===========================================
  // メイン処理
  // ===========================================
  void onTimer();
  bool subscribeMessageAvailable();
  
  // ===========================================
  // パラメータ変更時のコールバック
  // ===========================================
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // ===========================================
  // Pure Pursuit制御関数
  // ===========================================
  std::optional<AckermannControlCommand> generateControlCommand();
  std::optional<TrajectoryPoint> calcTargetPoint() const;
  
  // ===========================================
  // 曲率計算・道路タイプ判定
  // ===========================================
  double calculateCurvature(
    const TrajectoryPoint& p1,
    const TrajectoryPoint& p2, 
    const TrajectoryPoint& p3);
    
  double getAverageCurvatureAhead(size_t current_idx);
  
  RoadType classifyRoadType(double curvature);
  
  // ===========================================
  // 道路タイプ別の調整係数取得
  // ===========================================
  void getControlFactors(
    RoadType road_type, 
    double& lookahead_factor, 
    double& speed_limit_factor);
  
  // ===========================================
  // 平滑化・ステアリング状態考慮
  // ===========================================
  double smoothLookaheadFactor(double target_factor);
  
  double adjustFactorForCorneringState(
    double base_factor, 
    double current_steering,
    RoadType road_type);
  
  // ===========================================
  // コールバックハンドル
  // ===========================================
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
