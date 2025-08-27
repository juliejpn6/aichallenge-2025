/**
 * 改良版高度レーシング制御システム - エラー修正版
 * 
 * 修正点：
 * 1. AckermannControlCommandのメンバー名修正
 * 2. 未使用変数・パラメータの整理
 * 3. コンパイルエラーの解消
 */
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <memory>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace improved_racing_control
{

using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Odometry = nav_msgs::msg::Odometry;
using PointStamped = geometry_msgs::msg::PointStamped;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

// ============================================
// コーナー分析と車両状態構造体
// ============================================

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

struct ControlOutput {
    double steering_angle;      // ステアリング角度（ラジアン）
    double target_speed;        // 目標速度（m/s）
    double lookahead_distance;  // 先読み距離（m）
    CornerAnalysis corner_info; // コーナー情報
    double stability_factor;    // 安定性係数
};

// ============================================
// 改良版コーナー解析クラス
// ============================================

class ImprovedCornerAnalyzer
{
private:
    double max_lateral_g_;
    double max_speed_kmh_;
    std::deque<double> curvature_history_;
    
public:
    ImprovedCornerAnalyzer(double max_lateral_g, double max_speed_kmh) 
        : max_lateral_g_(max_lateral_g), max_speed_kmh_(max_speed_kmh) {
        curvature_history_.resize(8, 0.0);
    }
    
    CornerAnalysis analyzeCorner(const Trajectory& trajectory, 
                                size_t current_idx,
                                double current_speed) {
        CornerAnalysis analysis;
        
        const size_t analysis_window = 15;
        
        // 曲率計算（平滑化版）
        analysis.curvature = calculateSmoothedCurvature(trajectory, current_idx, analysis_window);
        
        // コーナータイプ判定
        analysis.type = classifyCornerType(analysis.curvature);
        
        // コーナーフェーズ判定
        analysis.phase = determineCornerPhase(trajectory, current_idx, analysis.curvature);
        
        // 最適速度計算
        analysis.optimal_speed = calculateOptimalSpeed(analysis.type, analysis.curvature, current_speed);
        
        // 先読み距離計算
        analysis.lookahead_distance = calculateLookaheadDistance(analysis.phase, current_speed);
        
        // 戦略情報
        analysis.strategy_info = generateStrategyInfo(analysis);
        
        return analysis;
    }

private:
    double calculateSmoothedCurvature(const Trajectory& trajectory, 
                                    size_t idx, 
                                    size_t window_size) {
        if (trajectory.points.empty() || idx >= trajectory.points.size()) {
            return 0.0;
        }
        
        double raw_curvature = 0.0;
        size_t valid_points = 0;
        
        for (size_t i = 0; i < window_size && (idx + i) < trajectory.points.size() - 1; ++i) {
            size_t curr_idx = idx + i;
            if (curr_idx > 0 && curr_idx < trajectory.points.size() - 1) {
                double curvature = calculatePointCurvature(trajectory, curr_idx);
                raw_curvature += curvature;
                valid_points++;
            }
        }
        
        if (valid_points > 0) {
            raw_curvature /= valid_points;
        }
        
        // 履歴に追加して移動平均
        curvature_history_.pop_front();
        curvature_history_.push_back(raw_curvature);
        
        double smoothed = std::accumulate(curvature_history_.begin(), curvature_history_.end(), 0.0) 
                         / curvature_history_.size();
        
        return smoothed;
    }
    
    double calculatePointCurvature(const Trajectory& trajectory, size_t idx) {
        if (idx == 0 || idx >= trajectory.points.size() - 1) {
            return 0.0;
        }
        
        const auto& p1 = trajectory.points[idx - 1].pose.position;
        const auto& p2 = trajectory.points[idx].pose.position;
        const auto& p3 = trajectory.points[idx + 1].pose.position;
        
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;
        
        double cross_product = dx1 * dy2 - dy1 * dx2;
        double ds1 = std::hypot(dx1, dy1);
        double ds2 = std::hypot(dx2, dy2);
        
        if (ds1 < 1e-6 || ds2 < 1e-6) {
            return 0.0;
        }
        
        return 2.0 * cross_product / (ds1 * ds2 * (ds1 + ds2));
    }
    
    CornerAnalysis::CornerType classifyCornerType(double curvature) {
        double abs_curvature = std::abs(curvature);
        
        if (abs_curvature < 0.01) return CornerAnalysis::STRAIGHT;
        if (abs_curvature < 0.03) return CornerAnalysis::GENTLE_CURVE;
        if (abs_curvature < 0.08) return CornerAnalysis::NORMAL_CORNER;
        if (abs_curvature < 0.15) return CornerAnalysis::TIGHT_CORNER;
        return CornerAnalysis::HAIRPIN;
    }
    
    CornerAnalysis::CornerPhase determineCornerPhase(const Trajectory& trajectory, 
                                                   size_t idx, 
                                                   double curvature) {
        if (trajectory.points.empty()) {
            return CornerAnalysis::STRAIGHT_OUT;
        }
        
        double abs_curvature = std::abs(curvature);
        
        // 前後の曲率変化を解析
        double future_curvature = 0.0;
        
        // 未来の曲率（5点先を見る）
        const size_t future_points = 5;
        if (idx + future_points < trajectory.points.size()) {
            future_curvature = calculatePointCurvature(trajectory, idx + future_points);
        }
        
        // フェーズ判定ロジック（改良版）
        // 【重要】出口検出を早期化してオーバーステア防止
        if (abs_curvature < 0.015) {
            return CornerAnalysis::STRAIGHT_OUT;
        }
        
        // 曲率減少傾向ならEXITフェーズ（早期検出）
        if (std::abs(future_curvature) < abs_curvature * 0.7) {
            return CornerAnalysis::EXIT;
        }
        
        // 曲率増加傾向ならENTRYまたはAPPROACH
        if (std::abs(future_curvature) > abs_curvature * 1.3) {
            return abs_curvature > 0.05 ? CornerAnalysis::ENTRY : CornerAnalysis::APPROACH;
        }
        
        // 曲率安定ならAPEX
        return CornerAnalysis::APEX;
    }
    
    double calculateOptimalSpeed(CornerAnalysis::CornerType type, 
                               double curvature, 
                               double current_speed) {
        const double safety_margin = 0.85;
        
        double base_speed = max_speed_kmh_ / 3.6;  // km/h → m/s変換
        double optimal_speed;
        
        // コーナータイプ別速度設定
        switch (type) {
            case CornerAnalysis::STRAIGHT:
                optimal_speed = base_speed * 1.0;
                break;
            case CornerAnalysis::GENTLE_CURVE:
                optimal_speed = base_speed * 0.85;
                break;
            case CornerAnalysis::NORMAL_CORNER:
                optimal_speed = base_speed * 0.70;
                break;
            case CornerAnalysis::TIGHT_CORNER:
                optimal_speed = base_speed * 0.55;
                break;
            case CornerAnalysis::HAIRPIN:
                optimal_speed = base_speed * 0.40;
                break;
            default:
                optimal_speed = base_speed * 0.60;
                break;
        }
        
        // 曲率ベース調整
        if (std::abs(curvature) > 0.01) {
            double curvature_limited_speed = std::sqrt(max_lateral_g_ * 9.81 / std::abs(curvature));
            optimal_speed = std::min(optimal_speed, curvature_limited_speed);
        }
        
        // 安全マージン適用
        optimal_speed *= safety_margin;
        
        // 急激な変化抑制
        double change_limit = 0.20;
        double max_change = current_speed * change_limit;
        
        if (optimal_speed > current_speed + max_change) {
            optimal_speed = current_speed + max_change;
        } else if (optimal_speed < current_speed - max_change) {
            optimal_speed = current_speed - max_change;
        }
        
        return std::max(optimal_speed, 2.0);
    }
    
    double calculateLookaheadDistance(CornerAnalysis::CornerPhase phase, double current_speed) {
        const double base_distance = 8.0;
        const double speed_factor = 0.4;
        
        double base_lookahead = base_distance + current_speed * speed_factor;
        
        double phase_multiplier;
        switch (phase) {
            case CornerAnalysis::APPROACH:
                phase_multiplier = 1.0;
                break;
            case CornerAnalysis::ENTRY:
                phase_multiplier = 0.85;
                break;
            case CornerAnalysis::APEX:
                phase_multiplier = 0.75;
                break;
            case CornerAnalysis::EXIT:
                phase_multiplier = 1.1;      // 【重要】出口で長く
                break;
            case CornerAnalysis::STRAIGHT_OUT:
                phase_multiplier = 1.3;      // 【重要】直線復帰で最長
                break;
            default:
                phase_multiplier = 1.0;
                break;
        }
        
        double final_distance = base_lookahead * phase_multiplier;
        return std::clamp(final_distance, 5.0, 25.0);
    }
    
    std::string generateStrategyInfo(const CornerAnalysis& analysis) {
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
// 改良版レーシング制御クラス
// ============================================

class ImprovedRacingController
{
private:
    double wheel_base_;
    std::deque<double> steering_history_;
    std::deque<double> curvature_history_;
    double last_steering_angle_;
    
    double max_steering_rate_;
    double stability_factor_;
    double oversteer_prevention_gain_;
    
public:
    ImprovedRacingController(double wheel_base) 
        : wheel_base_(wheel_base), last_steering_angle_(0.0) {
        steering_history_.resize(7, 0.0);
        curvature_history_.resize(10, 0.0);
        
        max_steering_rate_ = 1.2;
        stability_factor_ = 0.75;
        oversteer_prevention_gain_ = 0.6;
    }
    
    ControlOutput calculateControl(const CornerAnalysis& analysis,
                                 const Trajectory& trajectory,
                                 size_t current_idx,
                                 const geometry_msgs::msg::Pose& current_pose,
                                 double current_speed) {
        ControlOutput output;
        
        output.corner_info = analysis;
        output.target_speed = analysis.optimal_speed;
        output.lookahead_distance = analysis.lookahead_distance;
        
        // 基本ステアリング計算
        output.steering_angle = calculateBasicSteering(trajectory, current_idx, 
                                                     current_pose, output.lookahead_distance);
        
        // オーバーステア防止制御
        output.steering_angle = applyOversteerPrevention(output.steering_angle, 
                                                       analysis, current_speed);
        
        // 安定性係数計算
        output.stability_factor = calculateStabilityFactor(analysis, current_speed);
        
        // 最終平滑化
        output.steering_angle = applyFinalSmoothing(output.steering_angle);
        
        return output;
    }

private:
    double calculateBasicSteering(const Trajectory& trajectory,
                                size_t current_idx,
                                const geometry_msgs::msg::Pose& current_pose,
                                double lookahead_distance) {
        if (trajectory.points.empty() || current_idx >= trajectory.points.size()) {
            return 0.0;
        }
        
        // 後輪位置計算
        double rear_x = current_pose.position.x -
                       wheel_base_ / 2.0 * std::cos(tf2::getYaw(current_pose.orientation));
        double rear_y = current_pose.position.y -
                       wheel_base_ / 2.0 * std::sin(tf2::getYaw(current_pose.orientation));
        
        // 先読み点検索
        size_t lookahead_idx = current_idx;
        double min_distance_diff = std::numeric_limits<double>::max();
        
        for (size_t i = current_idx; i < trajectory.points.size(); ++i) {
            double distance = std::hypot(trajectory.points[i].pose.position.x - rear_x,
                                       trajectory.points[i].pose.position.y - rear_y);
            double distance_diff = std::abs(distance - lookahead_distance);
            
            if (distance_diff < min_distance_diff) {
                min_distance_diff = distance_diff;
                lookahead_idx = i;
            }
            
            if (distance > lookahead_distance) {
                break;
            }
        }
        
        // ステアリング角度計算
        const auto& lookahead_point = trajectory.points[lookahead_idx];
        double alpha = std::atan2(lookahead_point.pose.position.y - rear_y,
                                lookahead_point.pose.position.x - rear_x) -
                      tf2::getYaw(current_pose.orientation);
        
        double actual_distance = std::hypot(lookahead_point.pose.position.x - rear_x,
                                          lookahead_point.pose.position.y - rear_y);
        
        if (actual_distance < 0.1) {
            return 0.0;
        }
        
        return std::atan2(2.0 * wheel_base_ * std::sin(alpha), actual_distance);
    }
    
    double applyOversteerPrevention(double raw_steering, 
                                  const CornerAnalysis& analysis,
                                  double current_speed) {
        
        // フェーズベースゲイン
        double phase_gain = getPhaseBasedGain(analysis.phase);
        
        // 速度適応ゲイン
        double speed_gain = calculateSpeedBasedGain(current_speed);
        
        // オーバーステア防止ゲイン
        double oversteer_gain = calculateOversteerPreventionGain(analysis);
        
        // 総合ゲイン適用
        double adjusted_steering = raw_steering * phase_gain * speed_gain * oversteer_gain;
        
        // 変化率制限
        double rate_limited_steering = applyRateLimit(adjusted_steering, current_speed);
        
        return rate_limited_steering;
    }
    
    double getPhaseBasedGain(CornerAnalysis::CornerPhase phase) {
        switch (phase) {
            case CornerAnalysis::APPROACH:
                return 1.0;      // 通常
            case CornerAnalysis::ENTRY:
                return 1.05;     // やや攻撃的
            case CornerAnalysis::APEX:
                return 1.0;      // 通常
            case CornerAnalysis::EXIT:
                return 0.65;     // 【重要】大幅減少（オーバーステア防止）
            case CornerAnalysis::STRAIGHT_OUT:
                return 0.45;     // 【重要】最大減少（直線安定化）
            default:
                return 1.0;
        }
    }
    
    double calculateSpeedBasedGain(double current_speed) {
        const double base_speed = 8.0;
        const double gain_reduction = 0.3;
        
        if (current_speed <= base_speed) {
            return 1.0;
        }
        
        double speed_factor = (current_speed - base_speed) / base_speed;
        double gain = 1.0 - gain_reduction * std::min(speed_factor, 2.0);
        
        return std::max(gain, 0.4);
    }
    
    double calculateOversteerPreventionGain(const CornerAnalysis& analysis) {
        // 曲率履歴更新
        curvature_history_.pop_front();
        curvature_history_.push_back(analysis.curvature);
        
        // 曲率変化の激しさを計算
        double curvature_variation = 0.0;
        for (size_t i = 1; i < curvature_history_.size(); ++i) {
            double diff = curvature_history_[i] - curvature_history_[i-1];
            curvature_variation += std::abs(diff);
        }
        curvature_variation /= (curvature_history_.size() - 1);
        
        // 振動が大きい場合はゲインを下げる
        double vibration_factor = 1.0;
        if (curvature_variation > 0.02) {
            vibration_factor = 0.7;
        }
        
        // コーナータイプ別調整
        double type_factor = 1.0;
        switch (analysis.type) {
            case CornerAnalysis::TIGHT_CORNER:
            case CornerAnalysis::HAIRPIN:
                type_factor = 0.8;
                break;
            default:
                type_factor = 1.0;
                break;
        }
        
        return vibration_factor * type_factor;
    }
    
    double applyRateLimit(double target_steering, double current_speed) {
        double speed_factor = std::min(current_speed / 10.0, 1.5);
        double actual_max_rate = max_steering_rate_ / speed_factor;
        
        double steering_diff = target_steering - last_steering_angle_;
        
        const double dt = 0.05;  // 20Hz前提
        double max_change = actual_max_rate * dt;
        
        if (std::abs(steering_diff) > max_change) {
            double limited_diff = std::copysign(max_change, steering_diff);
            target_steering = last_steering_angle_ + limited_diff;
        }
        
        last_steering_angle_ = target_steering;
        
        return target_steering;
    }
    
    double calculateStabilityFactor(const CornerAnalysis& analysis, double current_speed) {
        double base_stability = 0.8;
        
        switch (analysis.phase) {
            case CornerAnalysis::EXIT:
            case CornerAnalysis::STRAIGHT_OUT:
                base_stability = 0.95;
                break;
            default:
                base_stability = 0.8;
                break;
        }
        
        double speed_stability = std::min(current_speed / 15.0, 1.0);
        
        return base_stability + (1.0 - base_stability) * speed_stability;
    }
    
    double applyFinalSmoothing(double target_steering) {
        steering_history_.pop_front();
        steering_history_.push_back(target_steering);
        
        const double smoothing_weight = 0.3;
        
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        
        for (size_t i = 0; i < steering_history_.size(); ++i) {
            double weight = std::pow(smoothing_weight, steering_history_.size() - 1 - i);
            weighted_sum += steering_history_[i] * weight;
            weight_sum += weight;
        }
        
        return weighted_sum / weight_sum;
    }
};

// ============================================
// メインクラス実装
// ============================================

class ImprovedRacingControl : public rclcpp::Node
{
public:
    ImprovedRacingControl()
        : Node("improved_racing_control") {
        
        // パラメータ宣言
        wheel_base_ = declare_parameter<float>("wheel_base", 2.14);
        max_speed_kmh_ = declare_parameter<float>("max_speed_kmh", 35.0);
        max_lateral_g_ = declare_parameter<float>("max_lateral_g", 0.82);
        enable_debug_output_ = declare_parameter<bool>("enable_debug_output", true);
        steering_tire_angle_gain_ = declare_parameter<float>("steering_tire_angle_gain", 3.639);
        
        // 制御システム初期化
        corner_analyzer_ = std::make_unique<ImprovedCornerAnalyzer>(max_lateral_g_, max_speed_kmh_);
        racing_controller_ = std::make_unique<ImprovedRacingController>(wheel_base_);
        
        // ROS インターフェース設定
        setupRosInterface();
        
        RCLCPP_INFO(get_logger(), "改良版レーシング制御システム起動 - オーバーステア修正版");
        RCLCPP_INFO(get_logger(), "主な改良点: コーナー出口安定化、蛇行抑制、適応制御");
    }

private:
    void setupRosInterface() {
        // パブリッシャー
        pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
        pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
        pub_lookahead_point_ = create_publisher<PointStamped>("output/lookahead_point", 1);
        pub_strategy_info_ = create_publisher<std_msgs::msg::String>("output/strategy_info", 1);
        
        // サブスクライバー
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
        sub_kinematics_ = create_subscription<Odometry>(
            "input/kinematics", qos, 
            [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
        sub_trajectory_ = create_subscription<Trajectory>(
            "input/trajectory", qos,
            [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
        
        // タイマー（20Hz制御）
        using namespace std::literals::chrono_literals;
        timer_ = rclcpp::create_timer(this, get_clock(), 50ms,
                                    std::bind(&ImprovedRacingControl::onTimer, this));
    }
    
    void onTimer() {
        if (!subscribeMessageAvailable()) {
            return;
        }
        
        // 現在位置と速度取得
        const auto& current_pose = odometry_->pose.pose;
        double current_speed = std::hypot(odometry_->twist.twist.linear.x,
                                        odometry_->twist.twist.linear.y);
        
        // 最近傍点検索
        size_t current_idx = findNearestIndex(trajectory_->points, current_pose.position);
        
        // コーナー解析
        CornerAnalysis corner_analysis = corner_analyzer_->analyzeCorner(
            *trajectory_, current_idx, current_speed);
        
        // 制御計算
        ControlOutput control_output = racing_controller_->calculateControl(
            corner_analysis, *trajectory_, current_idx, current_pose, current_speed);
        
        // 制御コマンド生成
        auto cmd = createControlCommand(control_output, current_speed);
        
        // パブリッシュ
        pub_cmd_->publish(cmd);
        
        // デバッグ情報出力
        if (enable_debug_output_) {
            publishDebugInfo(control_output);
        }
    }
    
    bool subscribeMessageAvailable() {
        return odometry_ && trajectory_ && !trajectory_->points.empty();
    }
    
    AckermannControlCommand createControlCommand(const ControlOutput& output, 
                                               double current_speed) {
        AckermannControlCommand cmd;
        cmd.stamp = get_clock()->now();
        
        // ステアリング設定（修正：正しいメンバー名を使用）
        cmd.lateral.steering_tire_angle = output.steering_angle * steering_tire_angle_gain_;
        
        // 速度制御
        cmd.longitudinal.speed = output.target_speed;
        cmd.longitudinal.acceleration = std::clamp(
            (output.target_speed - current_speed) * 2.0, -3.0, 3.0);
        
        return cmd;
    }
    
    void publishDebugInfo(const ControlOutput& output) {
        // 戦略情報
        if (pub_strategy_info_->get_subscription_count() > 0) {
            auto strategy_msg = std_msgs::msg::String();
            strategy_msg.data = output.corner_info.strategy_info + 
                               " | 安定性:" + std::to_string(output.stability_factor);
            pub_strategy_info_->publish(strategy_msg);
        }
        
        // 定期的なログ出力（5秒間隔）
        static auto last_log_time = get_clock()->now();
        if ((get_clock()->now() - last_log_time).seconds() > 5.0) {
            RCLCPP_INFO(get_logger(), 
                "制御状態 | %s | ステア:%.3f | 速度:%.1f | 安定性:%.2f",
                output.corner_info.strategy_info.c_str(),
                output.steering_angle,
                output.target_speed * 3.6,  // m/s → km/h
                output.stability_factor);
            last_log_time = get_clock()->now();
        }
    }
    
private:
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
    
    // パラメータ
    float wheel_base_;
    float max_speed_kmh_;
    float max_lateral_g_;
    bool enable_debug_output_;
    float steering_tire_angle_gain_;
    
    // 制御システム
    std::unique_ptr<ImprovedCornerAnalyzer> corner_analyzer_;
    std::unique_ptr<ImprovedRacingController> racing_controller_;
};

}  // namespace improved_racing_control

// ============================================
// メイン関数
// ============================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<improved_racing_control::ImprovedRacingControl>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("improved_racing_control"), 
                    "エラーが発生しました: %s", e.what());
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}
