/**
 * 高度レーシングMPC制御システム - 修正版実装ファイル
 * Autoware型システム対応版
 */
#include "simple_pure_pursuit/enhanced_racing_control.hpp"
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace enhanced_racing_control
{

// ============================================
// コーナー分析クラス実装（修正版）
// ============================================
class CornerAnalyzer
{
private:
    double max_lateral_g_;
    double max_speed_kmh_;
    double corner_detection_threshold_;
    
public:
    CornerAnalyzer(double max_g, double max_speed) 
        : max_lateral_g_(max_g), max_speed_kmh_(max_speed), corner_detection_threshold_(0.02) {}
    
    // 修正：Trajectoryの型を直接使用
    CornerAnalysis analyzeCorner(const Trajectory& trajectory, 
                               size_t current_idx, double current_speed_mps)
    {
        CornerAnalysis analysis;
        
        // 軌道点数の確認
        if (trajectory.points.empty() || current_idx >= trajectory.points.size()) {
            // デフォルト値を返す
            analysis.type = CornerAnalysis::STRAIGHT;
            analysis.phase = CornerAnalysis::STRAIGHT_OUT;
            analysis.curvature = 0.0;
            analysis.optimal_speed = max_speed_kmh_ / 3.6;
            analysis.lookahead_distance = 5.0;
            analysis.strategy_info = "直線-デフォルト";
            return analysis;
        }
        
        // 曲率計算
        analysis.curvature = calculateCurrentCurvature(trajectory, current_idx);
        
        // コーナータイプ判定
        analysis.type = classifyCornerType(analysis.curvature);
        
        // コーナー段階判定
        analysis.phase = determineCornerPhase(trajectory, current_idx);
        
        // 最適速度計算
        analysis.optimal_speed = calculateOptimalSpeed(analysis.curvature, analysis.type, analysis.phase);
        
        // 先読み距離計算
        analysis.lookahead_distance = calculateLookaheadDistance(analysis.phase, current_speed_mps);
        
        // 戦略情報
        analysis.strategy_info = generateStrategyInfo(analysis);
        
        return analysis;
    }

private:
    double calculateCurrentCurvature(const Trajectory& trajectory, size_t idx)
    {
        if (idx == 0 || idx >= trajectory.points.size() - 1) return 0.0;
        
        const auto& p1 = trajectory.points[idx - 1].pose.position;
        const auto& p2 = trajectory.points[idx].pose.position;
        const auto& p3 = trajectory.points[idx + 1].pose.position;
        
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;
        
        double cross_product = dx1 * dy2 - dy1 * dx2;
        double norm1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double norm2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        
        if (norm1 > 0.01 && norm2 > 0.01) {
            return std::abs(2.0 * cross_product / (norm1 * norm2 * (norm1 + norm2)));
        }
        return 0.0;
    }
    
    CornerAnalysis::CornerType classifyCornerType(double curvature)
    {
        if (curvature < 0.02) return CornerAnalysis::STRAIGHT;
        else if (curvature < 0.05) return CornerAnalysis::GENTLE_CURVE;
        else if (curvature < 0.10) return CornerAnalysis::NORMAL_CORNER;
        else if (curvature < 0.20) return CornerAnalysis::TIGHT_CORNER;
        else return CornerAnalysis::HAIRPIN;
    }
    
    CornerAnalysis::CornerPhase determineCornerPhase(const Trajectory& trajectory, size_t idx)
    {
        // 安全性チェック
        if (idx < 5 || idx >= trajectory.points.size() - 5) {
            return CornerAnalysis::STRAIGHT_OUT;
        }
        
        double current_curvature = calculateCurrentCurvature(trajectory, idx);
        double future_curvature = calculateCurrentCurvature(trajectory, idx + 3);
        double past_curvature = calculateCurrentCurvature(trajectory, idx - 3);
        
        if (current_curvature < corner_detection_threshold_) {
            return (future_curvature > current_curvature * 1.5) ? 
                   CornerAnalysis::APPROACH : CornerAnalysis::STRAIGHT_OUT;
        } else if (future_curvature > current_curvature * 1.2) {
            return CornerAnalysis::ENTRY;
        } else if (current_curvature > past_curvature * 1.2) {
            return CornerAnalysis::EXIT;
        } else {
            return CornerAnalysis::APEX;
        }
    }
    
    double calculateOptimalSpeed(double curvature, CornerAnalysis::CornerType type, CornerAnalysis::CornerPhase phase)
    {
        if (curvature < 1e-6) return max_speed_kmh_ / 3.6;
        
        double radius = 1.0 / curvature;
        double base_speed = std::sqrt(max_lateral_g_ * 9.81 * radius);
        base_speed = std::min(base_speed, max_speed_kmh_ / 3.6);
        
        // タイプ別補正
        switch (type) {
            case CornerAnalysis::GENTLE_CURVE: base_speed *= 0.95; break;
            case CornerAnalysis::NORMAL_CORNER: base_speed *= 0.90; break;
            case CornerAnalysis::TIGHT_CORNER: base_speed *= 0.85; break;
            case CornerAnalysis::HAIRPIN: base_speed *= 0.80; break;
            default: break;
        }
        
        // 段階別補正
        switch (phase) {
            case CornerAnalysis::APPROACH: return base_speed * 1.05;
            case CornerAnalysis::ENTRY: return base_speed * 1.00;
            case CornerAnalysis::APEX: return base_speed * 0.97;
            case CornerAnalysis::EXIT: return base_speed * 1.08;
            case CornerAnalysis::STRAIGHT_OUT: return max_speed_kmh_ / 3.6;
            default: return base_speed;
        }
    }
    
    // 修正：型パラメータを削除
    double calculateLookaheadDistance(CornerAnalysis::CornerPhase phase, double speed)
    {
        double base_distance = speed * 1.5 + 2.0;  // 基本計算
        
        // 段階別調整
        switch (phase) {
            case CornerAnalysis::APPROACH: return base_distance * 1.5;  // 8.0m程度
            case CornerAnalysis::ENTRY: return base_distance * 0.8;     // 4.0m程度
            case CornerAnalysis::APEX: return base_distance * 0.6;      // 3.0m程度
            case CornerAnalysis::EXIT: return base_distance * 1.2;      // 6.0m程度
            case CornerAnalysis::STRAIGHT_OUT: return base_distance * 2.0; // 10.0m程度
            default: return base_distance;
        }
    }
    
    std::string generateStrategyInfo(const CornerAnalysis& analysis)
    {
        std::string type_str, phase_str;
        
        switch (analysis.type) {
            case CornerAnalysis::STRAIGHT: type_str = "直線"; break;
            case CornerAnalysis::GENTLE_CURVE: type_str = "緩カーブ"; break;
            case CornerAnalysis::NORMAL_CORNER: type_str = "通常コーナー"; break;
            case CornerAnalysis::TIGHT_CORNER: type_str = "タイトコーナー"; break;
            case CornerAnalysis::HAIRPIN: type_str = "ヘアピン"; break;
        }
        
        switch (analysis.phase) {
            case CornerAnalysis::APPROACH: phase_str = "進入"; break;
            case CornerAnalysis::ENTRY: phase_str = "入口"; break;
            case CornerAnalysis::APEX: phase_str = "中"; break;
            case CornerAnalysis::EXIT: phase_str = "出口"; break;
            case CornerAnalysis::STRAIGHT_OUT: phase_str = "直線復帰"; break;
        }
        
        return type_str + "-" + phase_str;
    }
};

// ============================================
// レーシング制御クラス実装（修正版）
// ============================================
class RacingController
{
private:
    double wheel_base_;
    std::deque<double> steering_history_;
    
public:
    RacingController(double wheel_base) : wheel_base_(wheel_base) {
        steering_history_.resize(5, 0.0);
    }
    
    ControlOutput calculateControl(const CornerAnalysis& analysis,
                                 const Trajectory& trajectory,
                                 size_t current_idx,
                                 const geometry_msgs::msg::Pose& current_pose,
                                 double current_speed)
    {
        ControlOutput output;
        
        // コーナー分析をそのまま使用
        output.corner_info = analysis;
        output.target_speed = analysis.optimal_speed;
        output.lookahead_distance = analysis.lookahead_distance;
        
        // ステアリング計算
        output.steering_angle = calculateSteering(trajectory, current_idx, current_pose, 
                                                output.lookahead_distance);
        
        // 段階別ステアリング補正
        output.steering_angle *= getSteeringGain(analysis.phase);
        
        // セルフアライニング効果適用
        output.steering_angle = applySelfAligningEffect(output.steering_angle, current_speed);
        
        return output;
    }

private:
    double calculateSteering(const Trajectory& trajectory,
                           size_t current_idx,
                           const geometry_msgs::msg::Pose& current_pose,
                           double lookahead_distance)
    {
        // 安全性チェック
        if (trajectory.points.empty() || current_idx >= trajectory.points.size()) {
            return 0.0;
        }
        
        // 後輪位置計算
        double rear_x = current_pose.position.x -
                       wheel_base_ / 2.0 * std::cos(tf2::getYaw(current_pose.orientation));
        double rear_y = current_pose.position.y -
                       wheel_base_ / 2.0 * std::sin(tf2::getYaw(current_pose.orientation));
        
        // 先読み点検索（修正版）
        size_t lookahead_idx = current_idx;
        for (size_t i = current_idx; i < trajectory.points.size(); ++i) {
            double distance = std::hypot(trajectory.points[i].pose.position.x - rear_x,
                                       trajectory.points[i].pose.position.y - rear_y);
            if (distance >= lookahead_distance) {
                lookahead_idx = i;
                break;
            }
            lookahead_idx = i;  // 最後の点を使用
        }
        
        // ステアリング角度計算
        const auto& lookahead_point = trajectory.points[lookahead_idx];
        double alpha = std::atan2(lookahead_point.pose.position.y - rear_y,
                                lookahead_point.pose.position.x - rear_x) -
                      tf2::getYaw(current_pose.orientation);
        
        return std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
    }
    
    double getSteeringGain(CornerAnalysis::CornerPhase phase)
    {
        switch (phase) {
            case CornerAnalysis::APPROACH: return 1.0;
            case CornerAnalysis::ENTRY: return 1.15;     // 攻撃的
            case CornerAnalysis::APEX: return 1.0;
            case CornerAnalysis::EXIT: return 0.9;       // 緩める
            case CornerAnalysis::STRAIGHT_OUT: return 0.75; // 自然
            default: return 1.0;
        }
    }
    
    double applySelfAligningEffect(double target_steering, double current_speed)
    {
        // 高速時のセルフアライニング効果
        double speed_kmh = current_speed * 3.6;
        double self_aligning_factor = 0.0;
        
        if (speed_kmh > 20.0) {
            self_aligning_factor = 0.12 * std::min(1.0, (speed_kmh - 20.0) / 15.0);
        }
        
        // 履歴更新
        steering_history_.pop_front();
        steering_history_.push_back(target_steering);
        
        // 平滑化
        double avg_steering = std::accumulate(steering_history_.begin(), steering_history_.end(), 0.0) 
                            / steering_history_.size();
        
        return target_steering * (1.0 - self_aligning_factor) + avg_steering * self_aligning_factor;
    }
};

// ============================================
// メインクラス実装（修正版）
// ============================================
EnhancedRacingControl::EnhancedRacingControl()
    : Node("enhanced_racing_control")
{
    // パラメータ宣言
    wheel_base_ = declare_parameter<float>("wheel_base", 2.14);
    max_speed_kmh_ = declare_parameter<float>("max_speed_kmh", 35.0);
    max_lateral_g_ = declare_parameter<float>("max_lateral_g", 0.82);
    enable_advanced_control_ = declare_parameter<bool>("enable_advanced_control", true);
    enable_debug_output_ = declare_parameter<bool>("enable_debug_output", true);
    
    // 高度制御システム初期化
    corner_analyzer_ = std::make_unique<CornerAnalyzer>(max_lateral_g_, max_speed_kmh_);
    racing_controller_ = std::make_unique<RacingController>(wheel_base_);
    
    // パブリッシャー
    pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
    pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
    pub_lookahead_point_ = create_publisher<PointStamped>("output/lookahead_point", 1);
    pub_strategy_info_ = create_publisher<std_msgs::msg::String>("output/strategy_info", 1);
    
    // サブスクライバー
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    sub_kinematics_ = create_subscription<Odometry>(
        "input/kinematics", qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
    sub_trajectory_ = create_subscription<Trajectory>(
        "input/trajectory", qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
    
    // タイマー
    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(this, get_clock(), 20ms, 
                                std::bind(&EnhancedRacingControl::onTimer, this));
    
    RCLCPP_INFO(get_logger(), "高度レーシング制御システム起動完了 - 33秒台達成モード");
}

void EnhancedRacingControl::onTimer()
{
    if (!subscribeMessageAvailable()) return;
    
    // 現在位置取得（修正版）
    size_t current_idx = motion_utils::findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
    double current_speed = odometry_->twist.twist.linear.x;
    
    // 高度制御計算（修正版）
    ControlOutput output;
    if (enable_advanced_control_) {
        auto corner_analysis = corner_analyzer_->analyzeCorner(*trajectory_, current_idx, current_speed);
        output = racing_controller_->calculateControl(corner_analysis, *trajectory_, 
                                                    current_idx, odometry_->pose.pose, current_speed);
    } else {
        // フォールバック: 基本制御
        output.target_speed = 35.0 / 3.6;
        output.lookahead_distance = 5.0;
        output.steering_angle = 0.0;
        output.corner_info.strategy_info = "基本制御モード";
    }
    
    // 制御コマンド生成
    AckermannControlCommand cmd;
    cmd.stamp = get_clock()->now();
    cmd.longitudinal.speed = output.target_speed;
    cmd.longitudinal.acceleration = 1.5 * (output.target_speed - current_speed);
    cmd.lateral.steering_tire_angle = output.steering_angle;
    
    // パブリッシュ
    pub_cmd_->publish(cmd);
    
    // デバッグ情報
    if (enable_debug_output_) {
        publishDebugInfo(output);
    }
}

bool EnhancedRacingControl::subscribeMessageAvailable()
{
    return odometry_ != nullptr && trajectory_ != nullptr && !trajectory_->points.empty();
}

void EnhancedRacingControl::publishDebugInfo(const ControlOutput& output)
{
    // 戦略情報パブリッシュ
    std_msgs::msg::String strategy_msg;
    strategy_msg.data = output.corner_info.strategy_info + 
                       " | Speed: " + std::to_string(output.target_speed * 3.6) + "km/h" +
                       " | Lookahead: " + std::to_string(output.lookahead_distance) + "m";
    pub_strategy_info_->publish(strategy_msg);
    
    // 定期ログ出力
    static int log_counter = 0;
    if (++log_counter % 100 == 0) {  // ログ頻度を下げる
        RCLCPP_INFO(get_logger(), "制御状況: %s", strategy_msg.data.c_str());
    }
}

}  // namespace enhanced_racing_control

// ============================================
// メイン関数
// ============================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<enhanced_racing_control::EnhancedRacingControl>());
    rclcpp::shutdown();
    return 0;
}
