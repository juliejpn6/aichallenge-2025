#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <string>
#include <filesystem>

/**
 * ゴールライン通過判定＆パラメータ変更ノード（拡張版）
 * 
 * 【機能】
 * - ゴールライン通過を検出してラップカウント
 * - 周回ごとにsimple_trajectory_generatorのcsv_pathを変更
 * - 周回ごとにsimple_pure_pursuitのlookahead_gainとexternal_target_velを変更
 * - 初期速度・増加幅・最高速度をlaunchファイルで設定可能
 * - 詳細なデバッグログ出力機能
 * 
 * 【パラメータ】
 * - initial_speed: 初期速度 (km/h) ※1周目の速度
 * - speed_increment: 周回ごとの速度増加幅 (km/h)
 * - max_speed: 最高速度の上限 (km/h)
 * - in_threshold: ゴールエリア進入判定距離 (m)
 * - out_threshold: ゴールエリア退出判定距離 (m)
 * - enable_debug: デバッグログの有効/無効 (true/false)
 * - debug_interval: デバッグログ出力間隔 (メッセージ受信回数)
 */

class GoalLineChecker : public rclcpp::Node
{
public:
  GoalLineChecker() : Node("goal_line_checker_node")
  {
    // ===========================================
    // パラメータ宣言と読み込み
    // ===========================================
    
    // 速度関連パラメータ (初期値はデフォルト、launchファイルで上書き可能)
    this->declare_parameter("initial_speed", 12);      // 初期速度 (km/h)
    this->declare_parameter("speed_increment", 2);     // 速度増加幅 (km/h)
    this->declare_parameter("max_speed", 32);          // 最高速度 (km/h)
    
    // 判定エリア閾値パラメータ
    this->declare_parameter("in_threshold", 2.0);      // 進入判定距離 (m)
    this->declare_parameter("out_threshold", 3.0);     // 退出判定距離 (m)
    
    // デバッグ関連パラメータ
    this->declare_parameter("enable_debug", false);    // デバッグログ有効化
    this->declare_parameter("debug_interval", 100);     // デバッグ出力間隔
    
    // パラメータを取得してメンバ変数に格納
    initial_speed_ = this->get_parameter("initial_speed").as_int();
    speed_increment_ = this->get_parameter("speed_increment").as_int();
    max_speed_ = this->get_parameter("max_speed").as_int();
    
    in_threshold_ = this->get_parameter("in_threshold").as_double();
    out_threshold_ = this->get_parameter("out_threshold").as_double();
    
    enable_debug_ = this->get_parameter("enable_debug").as_bool();
    debug_interval_ = this->get_parameter("debug_interval").as_int();
    
    // ===========================================
    // 初期状態の設定
    // ===========================================
    
    is_in_goal_area_ = false;
    lap_count_ = 0;               // ラップカウント (0から開始、1周目で1になる)
    goal_received_ = false;
    odom_count_ = 0;
    is_updating_params_ = false;
    
    // デバッグ用タイマー初期化
    last_debug_time_ = this->now();
    
    // ===========================================
    // パラメータクライアントの作成
    // ===========================================
    
    // Trajectory Generator用（CSV切り替え）
    trajectory_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, 
      "/planning/scenario_planning/simple_trajectory_generator"
    );
    
    // Pure Pursuit用（lookahead_gain, external_target_vel更新）
    pure_pursuit_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this,
      "/simple_pure_pursuit_node"
    );
    
    // パラメータクライアントの接続確認(デバッグ用)
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] パラメータクライアント作成: trajectory & pure_pursuit");
    }
    
    // Trajectory Generatorノードの存在確認
    RCLCPP_INFO(this->get_logger(), "simple_trajectory_generatorノードへの接続を待機中...");
    if (trajectory_param_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(this->get_logger(), "✅ simple_trajectory_generatorノードに接続成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ simple_trajectory_generatorノードへの接続タイムアウト");
      RCLCPP_ERROR(this->get_logger(), "   CSV切り替え機能が動作しない可能性があります");
    }
    
    // Pure Pursuitノードの存在確認
    RCLCPP_INFO(this->get_logger(), "simple_pure_pursuitノードへの接続を待機中...");
    if (pure_pursuit_param_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(this->get_logger(), "✅ simple_pure_pursuitノードに接続成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ simple_pure_pursuitノードへの接続タイムアウト");
      RCLCPP_ERROR(this->get_logger(), "   パラメータ動的調整機能が動作しない可能性があります");
    }
    
    // ===========================================
    // サブスクライバー設定
    // ===========================================
    
    // ゴール位置を受信するサブスクライバー
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal",
      rclcpp::QoS(1).transient_local(),
      std::bind(&GoalLineChecker::goalCallback, this, std::placeholders::_1)
    );
    
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] サブスクライバー作成: /planning/mission_planning/goal");
    }
    
    // 車両位置(オドメトリ)を受信するサブスクライバー
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state",
      10,
      std::bind(&GoalLineChecker::odomCallback, this, std::placeholders::_1)
    );
    
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] サブスクライバー作成: /localization/kinematic_state");
    }
    
    // ===========================================
    // 起動ログ
    // ===========================================
    
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "ゴールライン通過判定ノード 起動（拡張版）");
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "【速度設定】");
    RCLCPP_INFO(this->get_logger(), "  初期速度: %d km/h", initial_speed_);
    RCLCPP_INFO(this->get_logger(), "  速度増加幅: %d km/h/周", speed_increment_);
    RCLCPP_INFO(this->get_logger(), "  最高速度: %d km/h", max_speed_);
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "【ゴール判定設定】");
    RCLCPP_INFO(this->get_logger(), "  進入閾値: %.2f m", in_threshold_);
    RCLCPP_INFO(this->get_logger(), "  退出閾値: %.2f m", out_threshold_);
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "【機能】");
    RCLCPP_INFO(this->get_logger(), "  CSV切り替え: 有効");
    RCLCPP_INFO(this->get_logger(), "  lookahead_gain動的調整: 有効");
    RCLCPP_INFO(this->get_logger(), "  external_target_vel更新: 有効");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "【デバッグ設定】");
    RCLCPP_INFO(this->get_logger(), "  デバッグモード: %s", enable_debug_ ? "有効" : "無効");
    if (enable_debug_) {
      RCLCPP_INFO(this->get_logger(), "  デバッグ出力間隔: %d メッセージごと", debug_interval_);
    }
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), " ");
  }

private:
  // ===========================================
  // ゴール位置受信コールバック
  // ===========================================
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] goalCallback: ゴール位置メッセージ受信");
    }
    
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] ゴール座標取得: X=%.2f, Y=%.2f", goal_x_, goal_y_);
    }
    
    if (!goal_received_) {
      goal_received_ = true;
      RCLCPP_INFO(this->get_logger(), " ");
      RCLCPP_INFO(this->get_logger(), "✓✓✓ ゴール位置受信成功 ✓✓✓");
      RCLCPP_INFO(this->get_logger(), "  X座標: %.2f m", goal_x_);
      RCLCPP_INFO(this->get_logger(), "  Y座標: %.2f m", goal_y_);
      RCLCPP_INFO(this->get_logger(), " ");
      
      if (enable_debug_) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "[DEBUG] goal_received_ フラグをtrueに設定");
      }
    }
  }
  
  // ===========================================
  // 車両位置受信コールバック (ゴール判定処理)
  // ===========================================
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_count_++;
    
    // デバッグ: メッセージ受信カウント
    if (enable_debug_ && odom_count_ % debug_interval_ == 0) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] odomCallback: メッセージ受信回数 = %d", odom_count_);
    }
    
    // ゴール位置がまだ受信されていない場合は処理をスキップ
    if (!goal_received_) {
      if (odom_count_ % 10 == 0) {
        RCLCPP_WARN(this->get_logger(), 
                    "⚠️  車両位置は受信中ですが、ゴール位置がまだ設定されていません");
        
        if (enable_debug_) {
          RCLCPP_DEBUG(this->get_logger(), 
                       "[DEBUG] ゴール位置未受信のため処理スキップ (odom_count=%d)", 
                       odom_count_);
        }
      }
      return;
    }
    
    // 車両とゴール間の距離を計算
    double vehicle_x = msg->pose.pose.position.x;
    double vehicle_y = msg->pose.pose.position.y;
    
    double dx = vehicle_x - goal_x_;
    double dy = vehicle_y - goal_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // デバッグ用: 定期的に距離と状態を表示
    if (odom_count_ % debug_interval_ == 0) {
      if (enable_debug_) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "[DEBUG] 車両位置: (%.2f, %.2f), ゴール位置: (%.2f, %.2f)", 
                     vehicle_x, vehicle_y, goal_x_, goal_y_);
        RCLCPP_DEBUG(this->get_logger(), 
                     "[DEBUG] 距離: %.2f m, is_in_goal_area=%s, lap_count=%d", 
                     distance, 
                     is_in_goal_area_ ? "true" : "false", 
                     lap_count_);
      }
      
      RCLCPP_INFO(this->get_logger(), 
                  "[状態] 距離: %.2f m (進入閾値=%.2f, 退出閾値=%.2f) | ラップ: %d | エリア内: %s", 
                  distance, in_threshold_, out_threshold_, lap_count_,
                  is_in_goal_area_ ? "はい" : "いいえ");
    }
    
    // ===========================================
    // ゴールエリア進入判定
    // ===========================================
    if (!is_in_goal_area_) {
      // ゴールエリア外→エリア内へ進入
      if (distance < in_threshold_) {
        is_in_goal_area_ = true;
        
        if (enable_debug_) {
          RCLCPP_DEBUG(this->get_logger(), 
                       "[DEBUG] ゴールエリア進入検出: 距離 %.2f < 閾値 %.2f", 
                       distance, in_threshold_);
        }
        
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "→→→ ゴールエリア進入！ ←←←");
        RCLCPP_INFO(this->get_logger(), "  距離: %.2f m", distance);
        RCLCPP_INFO(this->get_logger(), "  進入閾値: %.2f m", in_threshold_);
        RCLCPP_INFO(this->get_logger(), " ");
      }
    }
    // ===========================================
    // ゴールライン通過判定 (ラップカウント)
    // ===========================================
    else {
      // ゴールエリア内→エリア外へ退出 = ゴールライン通過!
      if (distance > out_threshold_) {
        is_in_goal_area_ = false;
        lap_count_++;  // ラップカウントを1増やす
        
        if (enable_debug_) {
          RCLCPP_DEBUG(this->get_logger(), 
                       "[DEBUG] ゴールライン通過検出: 距離 %.2f > 閾値 %.2f", 
                       distance, out_threshold_);
          RCLCPP_DEBUG(this->get_logger(), 
                       "[DEBUG] ラップカウント更新: %d → %d", 
                       lap_count_ - 1, lap_count_);
        }
        
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁");
        RCLCPP_INFO(this->get_logger(), "  ゴールライン通過検出！");
        RCLCPP_INFO(this->get_logger(), "  現在のラップ数: %d", lap_count_);
        RCLCPP_INFO(this->get_logger(), "  退出距離: %.2f m (閾値: %.2f m)", 
                    distance, out_threshold_);
        RCLCPP_INFO(this->get_logger(), "🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁");
        RCLCPP_INFO(this->get_logger(), " ");
        
        // ★ 全パラメータ変更処理を非同期で実行
        if (!is_updating_params_) {
          if (enable_debug_) {
            RCLCPP_DEBUG(this->get_logger(), 
                         "[DEBUG] 全パラメータ変更処理を開始します");
          }
          updateAllParametersAsync();
        } else {
          RCLCPP_WARN(this->get_logger(), 
                     "⚠️  前回のパラメータ変更が完了していません。スキップします。");
          
          if (enable_debug_) {
            RCLCPP_DEBUG(this->get_logger(), 
                         "[DEBUG] is_updating_params_=true のため変更スキップ");
          }
        }
      }
    }
  }
  
  // ===========================================
  // ★★★ lookahead_gain 計算関数 ★★★
  // ===========================================
  
  /**
   * 最高速度からlookahead_gainを計算
   * 計算式: gain = 0.8 + (max_speed_ms - 3.0) × 0.08
   */
  double calculateLookaheadGain(double max_speed_ms)
  {
    return 0.8 + (max_speed_ms - 3.0) * 0.08;
  }
  
  // ===========================================
  // ★★★ 全パラメータ更新処理（非同期版）★★★
  // ===========================================
  void updateAllParametersAsync()
  {
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] updateAllParametersAsync: 関数開始 (lap_count=%d)", lap_count_);
    }
    
    // 変更中フラグを立てる
    is_updating_params_ = true;
    
    // -----------------------------------------------
    // 目標速度の計算
    // -----------------------------------------------
    int target_speed_kmh = initial_speed_ + (lap_count_ - 1) * speed_increment_;
    
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] 速度計算: %d + (%d - 1) * %d = %d km/h", 
                   initial_speed_, lap_count_, speed_increment_, target_speed_kmh);
    }
    
    // 最高速度を超えないように制限
    if (target_speed_kmh > max_speed_) {
      if (enable_debug_) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "[DEBUG] 速度制限適用: %d → %d km/h", target_speed_kmh, max_speed_);
      }
      
      target_speed_kmh = max_speed_;
      RCLCPP_WARN(this->get_logger(), 
                 "⚠️  計算速度が最高速度を超えました。%d km/hに制限します。", 
                 max_speed_);
    }
    
    // km/h → m/s 変換
    double target_speed_ms = target_speed_kmh / 3.6;
    
    // lookahead_gain を計算
    double new_lookahead_gain = calculateLookaheadGain(target_speed_ms);
    
    // -----------------------------------------------
    // CSVファイルパスの生成
    // -----------------------------------------------
    std::string csv_filename = "raceline_awsim_" + std::to_string(target_speed_kmh) + "km.csv";
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("simple_trajectory_generator");
    std::string csv_path = package_share_dir + "/data/" + csv_filename;
    
    if (enable_debug_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] CSVファイル名生成: %s", csv_filename.c_str());
      RCLCPP_DEBUG(this->get_logger(), 
                   "[DEBUG] CSVフルパス: %s", csv_path.c_str());
    }
    
    // ファイルの存在確認
    if (!std::filesystem::exists(csv_path)) {
      RCLCPP_ERROR(this->get_logger(), "❌ CSVファイルが見つかりません: %s", csv_path.c_str());
      is_updating_params_ = false;
      return;
    }
    
    // -----------------------------------------------
    // ログ出力
    // -----------------------------------------------
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "🔧🔧🔧 パラメータ変更開始 🔧🔧🔧");
    RCLCPP_INFO(this->get_logger(), "  ラップ数: %d", lap_count_);
    RCLCPP_INFO(this->get_logger(), "  目標速度: %d km/h (%.2f m/s)", target_speed_kmh, target_speed_ms);
    RCLCPP_INFO(this->get_logger(), "  lookahead_gain: %.3f", new_lookahead_gain);
    RCLCPP_INFO(this->get_logger(), "  新しいCSVファイル: %s", csv_filename.c_str());
    RCLCPP_INFO(this->get_logger(), " ");
    
    // ========================================
    // 1. Trajectory Generator のCSVパスを更新
    // ========================================
    if (trajectory_param_client_->service_is_ready()) {
      auto trajectory_params = std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("csv_path", csv_path)
      };
      
      trajectory_param_client_->set_parameters(
        trajectory_params,
        [this, csv_filename](
          std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future
        ) {
          try {
            auto results = future.get();
            
            if (!results.empty() && results[0].successful) {
              RCLCPP_INFO(this->get_logger(), "✅ Trajectory CSV変更成功: %s", csv_filename.c_str());
            } else {
              RCLCPP_ERROR(this->get_logger(), "❌ Trajectory CSV変更失敗");
              if (!results.empty()) {
                RCLCPP_ERROR(this->get_logger(), "  理由: %s", results[0].reason.c_str());
              }
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Trajectory CSV変更エラー: %s", e.what());
          }
        }
      );
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Trajectory Generatorサービスが利用できません");
    }
    
    // ========================================
    // 2. Pure Pursuit のパラメータを更新
    // ========================================
    if (pure_pursuit_param_client_->service_is_ready()) {
      auto pure_pursuit_params = std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("external_target_vel", target_speed_ms),
        rclcpp::Parameter("lookahead_gain", new_lookahead_gain)
      };
      
      pure_pursuit_param_client_->set_parameters(
        pure_pursuit_params,
        [this, target_speed_kmh, target_speed_ms, new_lookahead_gain](
          std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future
        ) {
          try {
            auto results = future.get();
            
            bool all_success = true;
            for (const auto& result : results) {
              if (!result.successful) {
                all_success = false;
                RCLCPP_ERROR(this->get_logger(), "❌ Pure Pursuitパラメータ変更失敗: %s", 
                            result.reason.c_str());
              }
            }
            
            if (all_success) {
              RCLCPP_INFO(this->get_logger(), "✅ Pure Pursuitパラメータ変更成功");
              RCLCPP_INFO(this->get_logger(), "  external_target_vel = %d km/h (%.2f m/s)", 
                         target_speed_kmh, target_speed_ms);
              RCLCPP_INFO(this->get_logger(), "  lookahead_gain = %.3f", new_lookahead_gain);
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Pure Pursuitパラメータ変更エラー: %s", e.what());
          }
          
          // 全ての変更完了
          is_updating_params_ = false;
          RCLCPP_INFO(this->get_logger(), " ");
          RCLCPP_INFO(this->get_logger(), "✅✅✅ 全パラメータ変更完了！ ✅✅✅");
          RCLCPP_INFO(this->get_logger(), " ");
        }
      );
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Pure Pursuitサービスが利用できません");
      is_updating_params_ = false;
    }
  }
  
  // ===========================================
  // メンバ変数
  // ===========================================
  
  // ROS2通信関連
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // パラメータクライアント
  std::shared_ptr<rclcpp::AsyncParametersClient> trajectory_param_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> pure_pursuit_param_client_;
  
  // ゴール位置
  double goal_x_;
  double goal_y_;
  bool goal_received_;
  
  // ゴール判定用閾値
  double in_threshold_;   // ゴールエリア進入判定距離 (m)
  double out_threshold_;  // ゴールエリア退出判定距離 (m)
  
  // 状態管理
  bool is_in_goal_area_;      // 現在ゴールエリア内にいるか
  int lap_count_;             // ラップカウント (周回数)
  int odom_count_;            // オドメトリ受信回数 (デバッグ用)
  bool is_updating_params_;   // パラメータ変更中フラグ
  
  // 速度パラメータ (launchファイルから読み込み)
  int initial_speed_;      // 初期速度 (km/h) - 1周目の速度
  int speed_increment_;    // 速度増加幅 (km/h) - 周回ごとの増加量
  int max_speed_;          // 最高速度 (km/h) - 上限値
  
  // デバッグ関連
  bool enable_debug_;      // デバッグログ有効フラグ
  int debug_interval_;     // デバッグログ出力間隔
  rclcpp::Time last_debug_time_;  // 最後のデバッグ出力時刻
};

// ===========================================
// メイン関数
// ===========================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // ログレベルをDEBUGに設定(環境変数で制御も可能)
  rcutils_logging_set_logger_level(
    "goal_line_checker_node", 
    RCUTILS_LOG_SEVERITY_DEBUG
  );
  
  rclcpp::spin(std::make_shared<GoalLineChecker>());
  rclcpp::shutdown();
  return 0;
}
