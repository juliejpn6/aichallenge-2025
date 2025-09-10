/**
 * 完全MPC制御システム - tf2完全独立版
 * trajectory追従性向上版
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <deque>

namespace advanced_mpc_control {

using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Odometry = nav_msgs::msg::Odometry;

// 独自のクォータニオン→ヨー角変換関数
inline double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 車両状態構造体
struct VehicleState {
    double x, y;          // 位置 [m]
    double yaw;           // ヨー角 [rad]
    double speed;         // 速度 [m/s]
    double steering;      // ステアリング角 [rad]
};

// MPC制御結果
struct MPCResult {
    double steering_angle;      // 最適ステアリング角
    double target_speed;        // 目標速度
    double tracking_error;      // 追従誤差
    std::string status_info;    // 状態情報
};

// 軌道追従特化MPC制御クラス
class TrajectoryMPCController
{
public:
    TrajectoryMPCController();
    void setParameters(double prediction_time, double tracking_weight, double smoothness_weight);
    MPCResult calculateOptimalControl(const VehicleState& current_state,
                                    const Trajectory& trajectory,
                                    size_t closest_idx);
    
    size_t findNearestTrajectoryIndex(const VehicleState& state, const Trajectory& trajectory);

private:
    // パラメータ
    double prediction_horizon_;
    double control_dt_;
    double weight_tracking_;
    double weight_smoothness_;
    double wheel_base_;
    double max_steering_angle_;
    double max_steering_rate_;
    
    // 安定性確保用
    std::deque<double> steering_history_;
    double previous_steering_;
    
    // 内部処理関数
    std::vector<VehicleState> generateReferenceStates(const Trajectory& trajectory, 
                                                     size_t start_idx, 
                                                     const VehicleState& current_state);
    double optimizeSteeringAngle(const VehicleState& current_state,
                               const std::vector<VehicleState>& reference_states);
    std::vector<VehicleState> predictVehicleMotion(const VehicleState& initial_state,
                                                  double steering_input,
                                                  int steps);
    double calculateTrackingCost(const std::vector<VehicleState>& predicted,
                               const std::vector<VehicleState>& reference);
    double ensureStability(double candidate_steering, double current_speed);
};

// メインMPC統合ノード
class AdvancedMPCNode : public rclcpp::Node
{
public:
    AdvancedMPCNode();

private:
    void controlTimerCallback();
    void publishDebugInfo(const MPCResult& result);
    VehicleState convertOdometryToState(const Odometry& odometry);
    
    // MPC制御器
    std::unique_ptr<TrajectoryMPCController> mpc_controller_;
    
    // ROS通信
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_debug_point_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_info_;
    rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // データ
    Odometry::SharedPtr current_odometry_;
    Trajectory::SharedPtr current_trajectory_;
    
    // パラメータ
    bool enable_mpc_;
    double mpc_prediction_time_;
    double mpc_tracking_weight_;
    double mpc_smoothness_weight_;
};

} // namespace advanced_mpc_control
