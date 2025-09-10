/**
 * 完全MPC制御システム実装 - tf2完全独立版
 * trajectory追従問題を根本的に解決
 */
#include "simple_pure_pursuit/advanced_mpc_controller.hpp"
#include <algorithm>
#include <numeric>

namespace advanced_mpc_control {

// ================================================
// TrajectoryMPCController 実装
// ================================================

TrajectoryMPCController::TrajectoryMPCController()
    : prediction_horizon_(1.8)        // 安定重視の適度な予測時間
    , control_dt_(0.05)               // 20Hz制御
    , weight_tracking_(20.0)          // 軌道追従重視
    , weight_smoothness_(12.0)        // 振動抑制重視
    , wheel_base_(2.14)               // カート標準
    , max_steering_angle_(70.0 * M_PI / 180.0)  // 70度（安定性重視）
    , max_steering_rate_(2.0)         // 2.0 rad/s（安定重視）
    , previous_steering_(0.0)
{
    steering_history_.resize(8, 0.0);  // 8ステップ履歴保持
}

void TrajectoryMPCController::setParameters(double prediction_time, 
                                          double tracking_weight, 
                                          double smoothness_weight)
{
    prediction_horizon_ = std::clamp(prediction_time, 0.8, 3.0);
    weight_tracking_ = std::clamp(tracking_weight, 5.0, 50.0);
    weight_smoothness_ = std::clamp(smoothness_weight, 5.0, 30.0);
    
    RCLCPP_INFO(rclcpp::get_logger("MPC"), 
        "MPC設定更新: 予測時間=%.1fs, 追従重み=%.1f, 滑らか重み=%.1f",
        prediction_horizon_, weight_tracking_, weight_smoothness_);
}

MPCResult TrajectoryMPCController::calculateOptimalControl(
    const VehicleState& current_state,
    const Trajectory& trajectory,
    size_t closest_idx)
{
    MPCResult result;
    result.steering_angle = 0.0;
    result.target_speed = current_state.speed;
    result.tracking_error = 0.0;
    result.status_info = "初期化";
    
    // 境界チェック
    if (trajectory.points.empty() || closest_idx >= trajectory.points.size()) {
        result.status_info = "軌道データ無効-安全制御";
        return result;
    }
    
    // ================================================
    // 1. 参照軌道生成（trajectory基準）
    // ================================================
    
    auto reference_states = generateReferenceStates(trajectory, closest_idx, current_state);
    
    if (reference_states.size() < 3) {
        result.status_info = "参照軌道不足-安全制御";
        return result;
    }
    
    // ================================================
    // 2. 最適ステアリング角計算
    // ================================================
    
    double optimal_steering = optimizeSteeringAngle(current_state, reference_states);
    
    // ================================================
    // 3. 安定性確保処理
    // ================================================
    
    double stable_steering = ensureStability(optimal_steering, current_state.speed);
    
    // ================================================
    // 4. 結果設定
    // ================================================
    
    result.steering_angle = stable_steering;
    result.target_speed = reference_states[0].speed;
    
    // 追従誤差計算
    result.tracking_error = std::hypot(
        current_state.x - reference_states[0].x,
        current_state.y - reference_states[0].y);
    
    // 状態情報生成
    char status_buffer[150];
    std::snprintf(status_buffer, sizeof(status_buffer),
        "MPC: ステア=%.1f度, 速度=%.1fkm/h, 追従誤差=%.2fm",
        stable_steering * 180.0 / M_PI,
        result.target_speed * 3.6,
        result.tracking_error);
    result.status_info = status_buffer;
    
    return result;
}

std::vector<VehicleState> TrajectoryMPCController::generateReferenceStates(
    const Trajectory& trajectory,
    size_t start_idx,
    const VehicleState& current_state)
{
    std::vector<VehicleState> reference_states;
    
    int prediction_steps = static_cast<int>(prediction_horizon_ / control_dt_);
    prediction_steps = std::min(prediction_steps, 35); // 計算量制限
    
    // 現在の移動距離を基準に軌道上の点を選択
    double step_distance = std::max(0.1, current_state.speed * control_dt_); // 最小距離保証
    
    for (int step = 0; step < prediction_steps; ++step)
    {
        double target_distance = step * step_distance;
        
        // trajectory上で対応する点を探索
        size_t target_idx = start_idx;
        double search_distance = 0.0;
        
        while (target_idx < trajectory.points.size() - 1)
        {
            const auto& curr_point = trajectory.points[target_idx];
            const auto& next_point = trajectory.points[target_idx + 1];
            
            double segment_length = std::hypot(
                next_point.pose.position.x - curr_point.pose.position.x,
                next_point.pose.position.y - curr_point.pose.position.y);
            
            if (search_distance + segment_length >= target_distance)
            {
                // 線形補間で正確な位置を計算
                double ratio = segment_length > 0.001 ? 
                              (target_distance - search_distance) / segment_length : 0.0;
                
                VehicleState ref_state;
                ref_state.x = curr_point.pose.position.x + 
                             ratio * (next_point.pose.position.x - curr_point.pose.position.x);
                ref_state.y = curr_point.pose.position.y + 
                             ratio * (next_point.pose.position.y - curr_point.pose.position.y);
                
                // 角度の線形補間（独自実装）
                double curr_yaw = extractYawFromQuaternion(curr_point.pose.orientation);
                double next_yaw = extractYawFromQuaternion(next_point.pose.orientation);
                double yaw_diff = next_yaw - curr_yaw;
                if (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
                if (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
                ref_state.yaw = curr_yaw + ratio * yaw_diff;
                
                // 速度補間
                ref_state.speed = curr_point.longitudinal_velocity_mps + 
                                 ratio * (next_point.longitudinal_velocity_mps - 
                                         curr_point.longitudinal_velocity_mps);
                ref_state.steering = 0.0;
                
                reference_states.push_back(ref_state);
                break;
            }
            
            search_distance += segment_length;
            target_idx++;
        }
        
        // 軌道終端に到達した場合
        if (target_idx >= trajectory.points.size() - 1 && !trajectory.points.empty()) {
            const auto& last_point = trajectory.points.back();
            VehicleState ref_state;
            ref_state.x = last_point.pose.position.x;
            ref_state.y = last_point.pose.position.y;
            ref_state.yaw = extractYawFromQuaternion(last_point.pose.orientation);
            ref_state.speed = last_point.longitudinal_velocity_mps;
            ref_state.steering = 0.0;
            reference_states.push_back(ref_state);
            break; // 終端到達で終了
        }
    }
    
    return reference_states;
}

double TrajectoryMPCController::optimizeSteeringAngle(
    const VehicleState& current_state,
    const std::vector<VehicleState>& reference_states)
{
    double best_steering = 0.0;
    double min_cost = std::numeric_limits<double>::max();
    
    // ================================================
    // 安全なグリッドサーチによる最適化
    // ================================================
    
    // ステアリング角候補の安全な範囲設定
    double steering_range = std::min(max_steering_angle_, 45.0 * M_PI / 180.0); // 45度以下
    int num_candidates = 15; // 計算負荷制限
    
    for (int i = 0; i < num_candidates; ++i)
    {
        double steering_candidate = -steering_range + 
            (2.0 * steering_range * i) / (num_candidates - 1);
        
        // ステアリング角速度制約チェック
        double steering_change = steering_candidate - previous_steering_;
        double max_change = max_steering_rate_ * control_dt_;
        if (std::abs(steering_change) > max_change) {
            continue; // 制約違反をスキップ
        }
        
        // 車両運動予測
        auto predicted_states = predictVehicleMotion(current_state, steering_candidate, 
                                                   std::min(int(reference_states.size()), 10));
        
        // コスト計算
        double tracking_cost = calculateTrackingCost(predicted_states, reference_states);
        double smoothness_cost = weight_smoothness_ * steering_change * steering_change;
        double effort_cost = 0.5 * steering_candidate * steering_candidate;
        
        double total_cost = tracking_cost + smoothness_cost + effort_cost;
        
        // 最適解更新
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            best_steering = steering_candidate;
        }
    }
    
    return best_steering;
}

std::vector<VehicleState> TrajectoryMPCController::predictVehicleMotion(
    const VehicleState& initial_state,
    double steering_input,
    int steps)
{
    std::vector<VehicleState> predicted;
    predicted.reserve(steps);
    
    VehicleState current = initial_state;
    
    for (int step = 0; step < steps; ++step)
    {
        // ステアリング応答（1次遅れ系）
        double steering_time_constant = 0.15; // 150ms応答
        double actual_steering = current.steering + 
            (steering_input - current.steering) * control_dt_ / steering_time_constant;
        
        // 車両運動学（bicycle model）
        double beta = std::atan2(wheel_base_ * std::tan(actual_steering), 2.0);
        
        VehicleState next_state;
        next_state.x = current.x + current.speed * std::cos(current.yaw + beta) * control_dt_;
        next_state.y = current.y + current.speed * std::sin(current.yaw + beta) * control_dt_;
        next_state.yaw = current.yaw + (current.speed * std::sin(beta) * 2.0 / wheel_base_) * control_dt_;
        next_state.speed = current.speed; // 速度一定仮定
        next_state.steering = actual_steering;
        
        predicted.push_back(next_state);
        current = next_state;
    }
    
    return predicted;
}

double TrajectoryMPCController::calculateTrackingCost(
    const std::vector<VehicleState>& predicted,
    const std::vector<VehicleState>& reference)
{
    double total_cost = 0.0;
    
    size_t min_size = std::min(predicted.size(), reference.size());
    
    for (size_t i = 0; i < min_size; ++i)
    {
        // 位置誤差
        double position_error = std::hypot(predicted[i].x - reference[i].x,
                                         predicted[i].y - reference[i].y);
        
        // 角度誤差
        double yaw_error = predicted[i].yaw - reference[i].yaw;
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        
        // 重み付きコスト
        total_cost += weight_tracking_ * (position_error * position_error + 
                                        0.3 * yaw_error * yaw_error);
    }
    
    return total_cost;
}

double TrajectoryMPCController::ensureStability(double candidate_steering, double current_speed)
{
    // 履歴平均化による振動抑制
    steering_history_.pop_front();
    steering_history_.push_back(candidate_steering);
    
    double average_steering = std::accumulate(steering_history_.begin(), 
                                            steering_history_.end(), 0.0) / steering_history_.size();
    
    // 速度に応じた安定化係数
    double speed_kmh = current_speed * 3.6;
    double stability_factor = 0.7; // 基本安定化係数
    
    if (speed_kmh > 30.0) {
        stability_factor = 0.8; // 高速時：より安定重視
    } else if (speed_kmh < 15.0) {
        stability_factor = 0.6; // 低速時：応答性重視
    }
    
    // 最終ステアリング角計算
    double stable_steering = candidate_steering * (1.0 - stability_factor) + 
                           average_steering * stability_factor;
    
    // 変化率制限
    double max_change = max_steering_rate_ * control_dt_;
    double steering_change = stable_steering - previous_steering_;
    steering_change = std::clamp(steering_change, -max_change, max_change);
    
    double final_steering = previous_steering_ + steering_change;
    previous_steering_ = final_steering;
    
    return final_steering;
}

size_t TrajectoryMPCController::findNearestTrajectoryIndex(const VehicleState& state, 
                                                         const Trajectory& trajectory)
{
    if (trajectory.points.empty()) return 0;
    
    size_t nearest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        const auto& point = trajectory.points[i];
        double distance = std::hypot(point.pose.position.x - state.x,
                                   point.pose.position.y - state.y);
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_idx = i;
        }
    }
    
    return nearest_idx;
}

// ================================================
// AdvancedMPCNode 実装
// ================================================

AdvancedMPCNode::AdvancedMPCNode() : Node("advanced_mpc_node")
{
    // ================================================
    // パラメータ宣言
    // ================================================
    
    declare_parameter("enable_mpc", true);
    declare_parameter("mpc_prediction_time", 1.8);
    declare_parameter("mpc_tracking_weight", 20.0);
    declare_parameter("mpc_smoothness_weight", 12.0);
    
    // パラメータ読み込み
    enable_mpc_ = get_parameter("enable_mpc").as_bool();
    mpc_prediction_time_ = get_parameter("mpc_prediction_time").as_double();
    mpc_tracking_weight_ = get_parameter("mpc_tracking_weight").as_double();
    mpc_smoothness_weight_ = get_parameter("mpc_smoothness_weight").as_double();
    
    // MPC制御器初期化
    mpc_controller_ = std::make_unique<TrajectoryMPCController>();
    mpc_controller_->setParameters(mpc_prediction_time_, mpc_tracking_weight_, mpc_smoothness_weight_);
    
    // ================================================
    // ROS通信設定
    // ================================================
    
    // パブリッシャー
    pub_control_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
    pub_debug_point_ = create_publisher<geometry_msgs::msg::PointStamped>("debug/mpc_target_point", 1);
    pub_debug_info_ = create_publisher<std_msgs::msg::String>("debug/mpc_status", 1);
    
    // サブスクライバー
    auto qos = rclcpp::QoS(1).durability_volatile().best_effort();
    sub_odometry_ = create_subscription<Odometry>(
        "input/kinematics", qos,
        [this](const Odometry::SharedPtr msg) { current_odometry_ = msg; });
    sub_trajectory_ = create_subscription<Trajectory>(
        "input/trajectory", qos,
        [this](const Trajectory::SharedPtr msg) { current_trajectory_ = msg; });
    
    // 制御タイマー（50Hz）
    control_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                     std::bind(&AdvancedMPCNode::controlTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), 
        "完全MPC制御システム起動 - 予測時間: %.1fs, 追従重み: %.1f, 振動抑制重み: %.1f",
        mpc_prediction_time_, mpc_tracking_weight_, mpc_smoothness_weight_);
}

void AdvancedMPCNode::controlTimerCallback()
{
    // データ有効性チェック
    if (!current_odometry_ || !current_trajectory_ || 
        current_trajectory_->points.empty())
    {
        // 安全停止
        AckermannControlCommand zero_cmd;
        zero_cmd.stamp = this->now();
        zero_cmd.lateral.steering_tire_angle = 0.0;
        zero_cmd.longitudinal.speed = 0.0;
        zero_cmd.longitudinal.acceleration = 0.0;
        pub_control_cmd_->publish(zero_cmd);
        return;
    }
    
    // ================================================
    // 車両状態変換
    // ================================================
    
    VehicleState current_state = convertOdometryToState(*current_odometry_);
    
    // ================================================
    // 最近接軌道点検索
    // ================================================
    
    size_t closest_idx = mpc_controller_->findNearestTrajectoryIndex(current_state, *current_trajectory_);
    
    // ================================================
    // MPC制御計算
    // ================================================
    
    MPCResult mpc_result;
    if (enable_mpc_)
    {
        mpc_result = mpc_controller_->calculateOptimalControl(current_state, *current_trajectory_, closest_idx);
    }
    else
    {
        // MPC無効時は安全な値
        mpc_result.steering_angle = 0.0;
        mpc_result.target_speed = current_state.speed;
        mpc_result.tracking_error = 0.0;
        mpc_result.status_info = "MPC無効-安全制御";
    }
    
    // ================================================
    // 制御指令発行
    // ================================================
    
    AckermannControlCommand control_cmd;
    control_cmd.stamp = this->now();
    control_cmd.lateral.steering_tire_angle = mpc_result.steering_angle;
    control_cmd.longitudinal.speed = mpc_result.target_speed;
    control_cmd.longitudinal.acceleration = 0.0;
    
    pub_control_cmd_->publish(control_cmd);
    
    // ================================================
    // デバッグ情報発行
    // ================================================
    
    publishDebugInfo(mpc_result);
}

VehicleState AdvancedMPCNode::convertOdometryToState(const Odometry& odometry)
{
    VehicleState state;
    state.x = odometry.pose.pose.position.x;
    state.y = odometry.pose.pose.position.y;
    state.yaw = extractYawFromQuaternion(odometry.pose.pose.orientation);  // 独自関数使用
    state.speed = odometry.twist.twist.linear.x;
    state.steering = 0.0;
    return state;
}

void AdvancedMPCNode::publishDebugInfo(const MPCResult& result)
{
    // ステータス情報
    std_msgs::msg::String debug_msg;
    debug_msg.data = result.status_info;
    pub_debug_info_->publish(debug_msg);
    
    // 10秒間隔で詳細ログ
    static auto last_log_time = this->now();
    if ((this->now() - last_log_time).seconds() > 10.0)
    {
        RCLCPP_INFO(get_logger(), 
            "MPC制御状態: %s, 追従誤差: %.2fm", 
            result.status_info.c_str(), result.tracking_error);
        last_log_time = this->now();
    }
}

} // namespace advanced_mpc_control

// ================================================
// ROS2ノードのメイン関数
// ================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<advanced_mpc_control::AdvancedMPCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
