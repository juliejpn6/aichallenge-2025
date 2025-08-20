// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

class CSVToTrajectory : public rclcpp::Node
{
public:
  CSVToTrajectory() : Node("csv_to_trajectory_node"),
    current_lap_(0),
    lap_time_(0.0),
    section_num_(0),
    current_trajectory_type_(TrajectoryType::PITLANE)  // 起動時はpitlane_trajectoryを使用
  {
    const auto rb_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    pub_ = this->create_publisher<Trajectory>("trajectory", rb_qos);
    set_parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CSVToTrajectory::on_parameter_event, this, std::placeholders::_1));

    // Parameters
    declare_parameter("csv_path", "");  // 通常走行用（後方互換性のため）
    declare_parameter("pitlane_trajectory", "");     // ピットレーン用（起動時使用）
    declare_parameter("raceline_trajectory", "");    // レースライン用（ラップ切り替わり後使用）
    z_ = declare_parameter<float>("z");
    use_lap_switching_ = declare_parameter<bool>("use_lap_switching", true);  // デフォルトでtrue
    
    // Subscribe to AWSIM status for lap information
    if (use_lap_switching_) {
      sub_awsim_status_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aichallenge/awsim/status", 10, 
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { 
          statusCallback(msg); 
        });
      RCLCPP_INFO(get_logger(), "Lap-based trajectory switching is ENABLED");
    } else {
      RCLCPP_INFO(get_logger(), "Lap-based trajectory switching is DISABLED");
    }

    // Load trajectories
    loadTrajectories();

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CSVToTrajectory::publish_trajectory, this));
  }

private:
  enum class TrajectoryType {
    PITLANE,   // ピットレーン用（起動時使用）
    RACELINE   // レースライン用（ラップ切り替わり後使用）
  };

  void loadTrajectories()
  {
    // ピットレーン用trajectoryの読み込み（起動時使用）
    std::string pitlane_csv_path = get_parameter("pitlane_trajectory").as_string();
    
    if (pitlane_csv_path.empty()) {
      RCLCPP_ERROR(get_logger(), "Pitlane trajectory path is not specified");
      return;
    }
    
    if (!loadCSVTrajectory(pitlane_csv_path, pitlane_trajectory_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load pitlane CSV file: %s", pitlane_csv_path.c_str());
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Loaded PITLANE trajectory from CSV with %zu points", pitlane_trajectory_.points.size());
    
    // レースライン用trajectoryの読み込み（ラップ切り替わり後使用）
    if (use_lap_switching_) {
      std::string raceline_csv_path = get_parameter("raceline_trajectory").as_string();
      if (!raceline_csv_path.empty()) {
        if (loadCSVTrajectory(raceline_csv_path, raceline_trajectory_)) {
          RCLCPP_INFO(get_logger(), "Loaded RACELINE trajectory from CSV with %zu points", raceline_trajectory_.points.size());
          has_raceline_trajectory_ = true;
        } else {
          RCLCPP_WARN(get_logger(), "Failed to load raceline CSV file: %s. Will use pitlane trajectory for all laps.", raceline_csv_path.c_str());
          has_raceline_trajectory_ = false;
        }
      } else {
        RCLCPP_WARN(get_logger(), "Raceline CSV path is not specified. Will use pitlane trajectory for all laps.");
        has_raceline_trajectory_ = false;
      }
    }
  }

  bool loadCSVTrajectory(const std::string & csv_path, Trajectory & trajectory)
  {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      return false;
    }
    
    std::string line;
    std::getline(file, line);  // Skip header
    
    trajectory.header.stamp = this->now();
    trajectory.header.frame_id = "map";
    trajectory.points.clear();
    
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;
      
      while (std::getline(ss, token, ',')) {
        values.push_back(std::stod(token));
      }
      
      if (values.size() != 8) {
        RCLCPP_WARN(get_logger(), "Invalid CSV line format, expected 8 values");
        continue;
      }
      
      TrajectoryPoint point;
      point.pose.position.x = values[0];
      point.pose.position.y = values[1];
      point.pose.position.z = z_;

      point.pose.orientation.x = values[3];
      point.pose.orientation.y = values[4];
      point.pose.orientation.z = values[5];
      point.pose.orientation.w = values[6];
      
      point.longitudinal_velocity_mps = values[7];
      
      point.lateral_velocity_mps = 0.0;
      point.acceleration_mps2 = 0.0;
      point.heading_rate_rps = 0.0;
      
      trajectory.points.push_back(point);
    }
    
    return !trajectory.points.empty();
  }

  void statusCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(), "AWSIM status message has insufficient data");
      return;
    }
    
    // メッセージから情報を取得
    const auto& data = msg->data;
    int new_lap = static_cast<int>(data[1]);      // ラップ番号
    lap_time_ = static_cast<float>(data[2]);      // 累積ラップタイム
    section_num_ = static_cast<int>(data[3]);     // セクション番号
    
    // ラップが変更された場合の処理
//    if (new_lap != current_lap_) {
    if (new_lap != 0) {
      int previous_lap = current_lap_;
      current_lap_ = new_lap;
      
      // Trajectory typeの決定
      TrajectoryType new_type = determineTrajectoryType(current_lap_);
      
      if (new_type != current_trajectory_type_) {
        current_trajectory_type_ = new_type;
        std::string type_name = (current_trajectory_type_ == TrajectoryType::RACELINE) ? "RACELINE" : "PITLANE";
        RCLCPP_INFO(get_logger(), "Lap changed from %d to %d: Switched to %s trajectory", 
                   previous_lap, current_lap_, type_name.c_str());
      }
    }
    
    // デバッグ情報を出力（5秒間隔で制限）
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
      "AWSIM Status - Lap: %d, Time: %.2f, Section: %d, Trajectory: %s", 
      current_lap_, lap_time_, section_num_,
      (current_trajectory_type_ == TrajectoryType::RACELINE) ? "RACELINE" : "PITLANE");
  }

  TrajectoryType determineTrajectoryType(int lap)
  {
    // ラップ0（起動時）はピットレーン用trajectory
    // ラップ1以降はレースライン用trajectory（利用可能な場合）
    if (lap == 0) {
      return TrajectoryType::PITLANE;
    } else if (has_raceline_trajectory_) {
      return TrajectoryType::RACELINE;
    } else {
      // レースライン用trajectoryが利用できない場合はピットレーン用を継続使用
      return TrajectoryType::PITLANE;
    }
  }

  Trajectory getCurrentTrajectory()
  {
    if (current_trajectory_type_ == TrajectoryType::RACELINE && has_raceline_trajectory_) {
      return raceline_trajectory_;
    } else {
      return pitlane_trajectory_;
    }
  }
  
  void publish_trajectory()
  {
    Trajectory current_trajectory = getCurrentTrajectory();
    
    if (current_trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "No trajectory points to publish");
      return;
    }
    
    current_trajectory.header.stamp = this->now();
    pub_->publish(current_trajectory);
    
    std::string type_name = (current_trajectory_type_ == TrajectoryType::RACELINE) ? "RACELINE" : "PITLANE";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 60000 /*ms*/, 
      "Published %s trajectory with %zu points (Lap %d)", 
      type_name.c_str(), current_trajectory.points.size(), current_lap_);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    for (const auto & param : parameters) {
      if (param.get_name() == "pitlane_trajectory") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string new_csv_path = param.as_string();
          
          if (!std::filesystem::exists(new_csv_path)) {
            RCLCPP_ERROR(get_logger(), "Pitlane trajectory file does not exist: '%s'", new_csv_path.c_str());
            result.successful = false;
            result.reason = "Pitlane trajectory file does not exist.";
            continue;
          }

          if (loadCSVTrajectory(new_csv_path, pitlane_trajectory_)) {
            RCLCPP_INFO(get_logger(), "Successfully reloaded PITLANE trajectory from CSV: %s with %zu points", 
                        new_csv_path.c_str(), pitlane_trajectory_.points.size());
          } else {
            RCLCPP_ERROR(get_logger(), "Failed to reload pitlane CSV file: %s", new_csv_path.c_str());
            result.successful = false;
            result.reason = "Failed to reload pitlane CSV file.";
          }
        }
      } else if (param.get_name() == "raceline_trajectory") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string new_csv_path = param.as_string();
          
          if (!new_csv_path.empty()) {
            if (!std::filesystem::exists(new_csv_path)) {
              RCLCPP_ERROR(get_logger(), "Raceline trajectory file does not exist: '%s'", new_csv_path.c_str());
              result.successful = false;
              result.reason = "Raceline trajectory file does not exist.";
              continue;
            }

            if (loadCSVTrajectory(new_csv_path, raceline_trajectory_)) {
              has_raceline_trajectory_ = true;
              RCLCPP_INFO(get_logger(), "Successfully reloaded RACELINE trajectory from CSV: %s with %zu points", 
                          new_csv_path.c_str(), raceline_trajectory_.points.size());
            } else {
              RCLCPP_ERROR(get_logger(), "Failed to reload raceline CSV file: %s", new_csv_path.c_str());
              has_raceline_trajectory_ = false;
              result.successful = false;
              result.reason = "Failed to reload raceline CSV file.";
            }
          } else {
            has_raceline_trajectory_ = false;
          }
        }
      } else if (param.get_name() == "z") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE || 
            param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          z_ = static_cast<float>(param.as_double());
          RCLCPP_INFO(get_logger(), "z parameter changed to %f", z_);
          // 既存のtrajectoryのz値を更新
          updateTrajectoryZ();
        } else {
          RCLCPP_WARN(get_logger(), "Parameter 'z' received with wrong type. Expected float/double.");
          result.successful = false;
          result.reason = "Invalid type for z parameter.";
        }
      }
    }
    return result;
  }

  void updateTrajectoryZ()
  {
    for (auto& point : pitlane_trajectory_.points) {
      point.pose.position.z = z_;
    }
    if (has_raceline_trajectory_) {
      for (auto& point : raceline_trajectory_.points) {
        point.pose.position.z = z_;
      }
    }
  }
  
  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_awsim_status_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Trajectory storage
  Trajectory pitlane_trajectory_;   // ピットレーン用（起動時使用）
  Trajectory raceline_trajectory_;  // レースライン用（ラップ切り替わり後使用）
  bool has_raceline_trajectory_ = false;
  
  // Parameters and state
  float z_;
  bool use_lap_switching_;
  OnSetParametersCallbackHandle::SharedPtr set_parameter_callback_handle_;
  
  // AWSIM status information
  int current_lap_;
  float lap_time_;
  int section_num_;
  TrajectoryType current_trajectory_type_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CSVToTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
