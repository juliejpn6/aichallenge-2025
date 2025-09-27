// PlotJuggler用データ配信システム - 修正版
// ROS2 Humble対応：lambda引数型を明示的に指定

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <deque>
#include <chrono>

class PlotJugglerDataPublisher : public rclcpp::Node
{
public:
  PlotJugglerDataPublisher() : Node("plotjuggler_data_publisher")
  {
    // パラメータ宣言
    this->declare_parameter("publish_rate_hz", 20.0);
    this->declare_parameter("delay_calculation_window", 100);
    
    // パラメータ取得
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    delay_window_size_ = this->get_parameter("delay_calculation_window").as_int();
    
    RCLCPP_INFO(this->get_logger(), "PlotJuggler用データ配信システムを起動中...");
    RCLCPP_INFO(this->get_logger(), "配信レート: %.1f Hz", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "遅延計算ウィンドウ: %d", delay_window_size_);
    
    // データ構造初期化
    initializeDataStructures();
    
    // ROSパブリッシャー設定
    setupPublishers();
    
    // ROSサブスクライバー設定
    setupSubscribers();
    
    // 定期配信タイマー
    auto publish_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    publish_timer_ = this->create_wall_timer(
      publish_period, std::bind(&PlotJugglerDataPublisher::publishPlotData, this));
    
    RCLCPP_INFO(this->get_logger(), "PlotJuggler用データ配信システムが正常に起動しました");
  }

private:
  // パラメータ
  double publish_rate_hz_;
  int delay_window_size_;
  
  // データ保存用構造体
  struct TimestampedData {
    rclcpp::Time timestamp;
    double value;
  };
  
  // データキュー（遅延計算用）
  std::deque<TimestampedData> target_steering_data_;
  std::deque<TimestampedData> actual_steering_data_;
  std::deque<TimestampedData> target_acceleration_data_;
  std::deque<TimestampedData> actual_acceleration_data_;
  std::deque<TimestampedData> velocity_data_;
  
  // 最新値保存用
  double latest_target_steering_ = 0.0;
  double latest_actual_steering_ = 0.0;
  double latest_target_acceleration_ = 0.0;
  double latest_actual_acceleration_ = 0.0;
  double latest_velocity_ = 0.0;
  
  // ROS関連
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  
  // パブリッシャー（PlotJuggler用）
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_target_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_actual_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_steering_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_target_acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_actual_acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_acceleration_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_steering_delay_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plot_acceleration_delay_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_summary_pub_;
  
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  void initializeDataStructures()
  {
    // データキューのサイズ制限設定
    // 各キューは delay_window_size_ の2倍まで保持（余裕をもたせる）
    RCLCPP_INFO(this->get_logger(), "データ構造を初期化しました");
  }
  
  void setupPublishers()
  {
    // QoS設定（PlotJuggler推奨設定）
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).durability_volatile().reliable();
    
    // 基本データ用パブリッシャー
    plot_target_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/target_steering", qos);
    plot_actual_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/actual_steering", qos);
    plot_steering_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/steering_error", qos);
    
    plot_target_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/target_acceleration", qos);
    plot_actual_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/actual_acceleration", qos);
    plot_acceleration_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/acceleration_error", qos);
    
    plot_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/velocity", qos);
    
    // 分析データ用パブリッシャー
    plot_steering_delay_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/steering_delay_ms", qos);
    plot_acceleration_delay_pub_ = this->create_publisher<std_msgs::msg::Float64>("/plot/acceleration_delay_ms", qos);
    
    // サマリーデータ用パブリッシャー
    plot_summary_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/plot/control_summary", qos);
    
    RCLCPP_INFO(this->get_logger(), "PlotJuggler用パブリッシャーを設定しました");
  }
  
  void setupSubscribers()
  {
    // QoS設定
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().best_effort();
    
    // 制御コマンド受信 - 型を明示的に指定
    control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", qos,
      [this](const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg) {
        auto current_time = this->get_clock()->now();
        latest_target_steering_ = msg->lateral.steering_tire_angle;
        latest_target_acceleration_ = msg->longitudinal.acceleration;
        
        // データキューに追加
        addTargetSteeringData(current_time, latest_target_steering_);
        addTargetAccelerationData(current_time, latest_target_acceleration_);
      });
    
    // アクチュエーション指令受信 - 型を明示的に指定
    actuation_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
      "/control/command/actuation_cmd", qos,
      [this](const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr msg) {
        // アクチュエーション指令の詳細処理
        auto current_time = this->get_clock()->now();
        // 追加のデータ処理が必要な場合はここに実装
      });
    
    // アクチュエーション状態受信 - 型を明示的に指定
    actuation_status_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
      "/vehicle/status/actuation_status", qos,
      [this](const tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr msg) {
        auto current_time = this->get_clock()->now();
        latest_actual_acceleration_ = msg->status.accel_status;
        
        // データキューに追加
        addActualAccelerationData(current_time, latest_actual_acceleration_);
      });
    
    // ステアリング状態受信 - 型を明示的に指定
    steering_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", qos,
      [this](const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
        auto current_time = this->get_clock()->now();
        latest_actual_steering_ = msg->steering_tire_angle;
        
        // データキューに追加
        addActualSteeringData(current_time, latest_actual_steering_);
      });
    
    // 車速受信 - 型を明示的に指定
    velocity_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", qos,
      [this](const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
        auto current_time = this->get_clock()->now();
        latest_velocity_ = msg->longitudinal_velocity;
        
        // データキューに追加
        addVelocityData(current_time, latest_velocity_);
      });
    
    // オドメトリ受信 - 型を明示的に指定
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        auto current_time = this->get_clock()->now();
        // オドメトリからの速度情報も活用
        if (msg->twist.twist.linear.x != 0.0) {
          latest_velocity_ = msg->twist.twist.linear.x;
          addVelocityData(current_time, latest_velocity_);
        }
      });
    
    RCLCPP_INFO(this->get_logger(), "ROSサブスクライバーを設定しました");
  }
  
  void addTargetSteeringData(const rclcpp::Time& timestamp, double value)
  {
    target_steering_data_.push_back({timestamp, value});
    limitQueueSize(target_steering_data_);
  }
  
  void addActualSteeringData(const rclcpp::Time& timestamp, double value)
  {
    actual_steering_data_.push_back({timestamp, value});
    limitQueueSize(actual_steering_data_);
  }
  
  void addTargetAccelerationData(const rclcpp::Time& timestamp, double value)
  {
    target_acceleration_data_.push_back({timestamp, value});
    limitQueueSize(target_acceleration_data_);
  }
  
  void addActualAccelerationData(const rclcpp::Time& timestamp, double value)
  {
    actual_acceleration_data_.push_back({timestamp, value});
    limitQueueSize(actual_acceleration_data_);
  }
  
  void addVelocityData(const rclcpp::Time& timestamp, double value)
  {
    velocity_data_.push_back({timestamp, value});
    limitQueueSize(velocity_data_);
  }
  
  void limitQueueSize(std::deque<TimestampedData>& queue)
  {
    // キューサイズ制限（メモリ使用量管理）
    const size_t max_size = static_cast<size_t>(delay_window_size_ * 2);
    while (queue.size() > max_size) {
      queue.pop_front();
    }
  }
  
  void publishPlotData()
  {
    // 基本データ配信
    publishBasicData();
    
    // 誤差データ計算・配信
    publishErrorData();
    
    // 遅延データ計算・配信
    publishDelayData();
    
    // サマリーデータ配信
    publishSummaryData();
  }
  
  void publishBasicData()
  {
    // 基本制御データの配信
    auto msg = std_msgs::msg::Float64();
    
    // ステアリング目標値
    msg.data = latest_target_steering_;
    plot_target_steering_pub_->publish(msg);
    
    // ステアリング実際値
    msg.data = latest_actual_steering_;
    plot_actual_steering_pub_->publish(msg);
    
    // 加速度目標値
    msg.data = latest_target_acceleration_;
    plot_target_acceleration_pub_->publish(msg);
    
    // 加速度実際値
    msg.data = latest_actual_acceleration_;
    plot_actual_acceleration_pub_->publish(msg);
    
    // 車速
    msg.data = latest_velocity_;
    plot_velocity_pub_->publish(msg);
  }
  
  void publishErrorData()
  {
    auto msg = std_msgs::msg::Float64();
    
    // ステアリング誤差
    double steering_error = latest_target_steering_ - latest_actual_steering_;
    msg.data = steering_error;
    plot_steering_error_pub_->publish(msg);
    
    // 加速度誤差
    double acceleration_error = latest_target_acceleration_ - latest_actual_acceleration_;
    msg.data = acceleration_error;
    plot_acceleration_error_pub_->publish(msg);
  }
  
  void publishDelayData()
  {
    auto msg = std_msgs::msg::Float64();
    
    // ステアリング遅延計算
    double steering_delay = calculateAverageDelay(target_steering_data_, actual_steering_data_);
    msg.data = steering_delay;
    plot_steering_delay_pub_->publish(msg);
    
    // 加速度遅延計算
    double acceleration_delay = calculateAverageDelay(target_acceleration_data_, actual_acceleration_data_);
    msg.data = acceleration_delay;
    plot_acceleration_delay_pub_->publish(msg);
  }
  
  void publishSummaryData()
  {
    // サマリーデータ作成（PlotJuggler用多次元データ）
    auto msg = std_msgs::msg::Float64MultiArray();
    
    msg.data.resize(10);
    msg.data[0] = latest_target_steering_;        // 目標ステアリング角
    msg.data[1] = latest_actual_steering_;        // 実際ステアリング角
    msg.data[2] = latest_target_acceleration_;    // 目標加速度
    msg.data[3] = latest_actual_acceleration_;    // 実際加速度
    msg.data[4] = latest_velocity_;               // 現在速度
    msg.data[5] = latest_target_steering_ - latest_actual_steering_;        // ステアリング誤差
    msg.data[6] = latest_target_acceleration_ - latest_actual_acceleration_; // 加速度誤差
    msg.data[7] = calculateAverageDelay(target_steering_data_, actual_steering_data_);     // ステアリング遅延
    msg.data[8] = calculateAverageDelay(target_acceleration_data_, actual_acceleration_data_); // 加速度遅延
    msg.data[9] = this->get_clock()->now().seconds(); // タイムスタンプ
    
    plot_summary_pub_->publish(msg);
  }
  
  double calculateAverageDelay(const std::deque<TimestampedData>& target_data, 
                              const std::deque<TimestampedData>& actual_data)
  {
    // 遅延計算アルゴリズム（簡易版）
    if (target_data.empty() || actual_data.empty()) {
      return 0.0;
    }
    
    // 最新のデータポイント間の時間差を計算
    auto latest_target_time = target_data.back().timestamp;
    auto latest_actual_time = actual_data.back().timestamp;
    
    // 遅延をミリ秒で計算
    double delay_seconds = (latest_actual_time - latest_target_time).seconds();
    return std::abs(delay_seconds) * 1000.0; // ミリ秒変換
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<PlotJugglerDataPublisher>();
    
    RCLCPP_INFO(node->get_logger(), "PlotJuggler用データ配信システムを開始します");
    
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("plotjuggler_data_publisher"), 
                 "エラーが発生しました: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
