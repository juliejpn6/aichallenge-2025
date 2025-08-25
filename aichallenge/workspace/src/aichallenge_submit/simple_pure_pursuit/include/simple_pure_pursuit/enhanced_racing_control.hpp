/**
 * 高度レーシングMPC制御システム - ヘッダーファイル
 * お台場カート場 33秒台達成用
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <vector>
#include <deque>

namespace enhanced_racing_control
{

using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Odometry = nav_msgs::msg::Odometry;
using PointStamped = geometry_msgs::msg::PointStamped;

// コーナー分析結果構造体
struct CornerAnalysis {
    enum CornerType { STRAIGHT, GENTLE_CURVE, NORMAL_CORNER, TIGHT_CORNER, HAIRPIN };
    enum CornerPhase { APPROACH, ENTRY, APEX, EXIT, STRAIGHT_OUT };
    
    CornerType type;
    CornerPhase phase;
    double curvature;
    double optimal_speed;
    double lookahead_distance;
    std::string strategy_info;
};

// 制御出力構造体
struct ControlOutput {
    double steering_angle;
    double target_speed;
    double lookahead_distance;
    CornerAnalysis corner_info;
};

// 前方宣言
class CornerAnalyzer;
class RacingController;

class EnhancedRacingControl : public rclcpp::Node
{
public:
    EnhancedRacingControl();
    ~EnhancedRacingControl() = default;

private:
    void onTimer();
    bool subscribeMessageAvailable();
    void publishDebugInfo(const ControlOutput& output);

    // ROS インターフェース
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
    rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_strategy_info_;
    rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
    rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;

    Odometry::SharedPtr odometry_;
    Trajectory::SharedPtr trajectory_;

    // 制御パラメータ
    float wheel_base_;
    float max_speed_kmh_;
    float max_lateral_g_;
    bool enable_advanced_control_;
    bool enable_debug_output_;

    // 高度制御システム
    std::unique_ptr<CornerAnalyzer> corner_analyzer_;
    std::unique_ptr<RacingController> racing_controller_;
};

}  // namespace enhanced_racing_control
