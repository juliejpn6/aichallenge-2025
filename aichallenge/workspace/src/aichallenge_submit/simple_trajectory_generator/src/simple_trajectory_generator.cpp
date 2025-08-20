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
    current_trajectory_type_(TrajectoryType::PITLANE),
    has_switched_to_raceline_(false),
    switching_in_progress_(false),
    publish_active_(true)  // 常にpublishをアクティブに
  {
    const auto rb_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    pub_ = this->create_publisher<Trajectory>("trajectory", rb_qos);
    set_parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CSVToTrajectory::on_parameter_event, this, std::placeholders::_1));

    // Parameters
    declare_parameter("csv_path", "");
    declare_parameter("pitlane_trajectory", "");
    declare_parameter("raceline_trajectory", "");
    z_ = declare_parameter<float>("z");
    use_lap_switching_ = declare_parameter<bool>("use_lap_switching", true);
    
    // Subscribe to AWSIM status for lap information
    if (use_lap_switching_) {
      sub_awsim_status_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/aichallenge/awsim/status", 10, 
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { 
          statusCallback(msg); 
        });
      RCLCPP_INFO(get_logger(), "Lap-based trajectory switching is ENABLED (continuous publishing, seamless switching)");
    } else {
      RCLCPP_INFO(get_logger(), "Lap-based trajectory switching is DISABLED");
    }

    // Load trajectories
    loadTrajectories();

    // 継続的にtrajectoryをpublish（高頻度で確実に）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 50ms間隔（20Hz）で高頻度publish
      std::bind(&CSVToTrajectory::publish_trajectory, this));
      
    RCLCPP_INFO(get_logger(), "Trajectory publisher initialized with 20Hz continuous publishing");
  }

private:
  enum class TrajectoryType {
    PITLANE,   
    RACELINE   
  };

  void loadTrajectories()
  {
    // Pitlane trajectory loading
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
    
    // Raceline trajectory loading
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
    
    const auto& data = msg->data;
    int new_lap = static_cast<int>(data[1]);
    lap_time_ = static_cast<float>(data[2]);
    section_num_ = static_cast<int>(data[3]);
    
    // Lap change detection
    bool lap_changed = (new_lap != current_lap_);
    
    if (lap_changed) {
      int previous_lap = current_lap_;
      current_lap_ = new_lap;
      
      // Critical: 0 -> 1 lap transition (immediate seamless switch)
      if (previous_lap == 0 && new_lap == 1 && !has_switched_to_raceline_) {
        performSeamlessSwitch();
      } else {
        RCLCPP_INFO(get_logger(), "Lap changed from %d to %d: Continuing with %s trajectory (seamless)", 
                   previous_lap, current_lap_,
                   (current_trajectory_type_ == TrajectoryType::RACELINE) ? "RACELINE" : "PITLANE");
      }
    }
    
    // Debug info (reduced frequency)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, 
      "Status - Lap: %d, Time: %.2f, Section: %d, Trajectory: %s, Publishing: %s", 
      current_lap_, lap_time_, section_num_,
      (current_trajectory_type_ == TrajectoryType::RACELINE) ? "RACELINE" : "PITLANE",
      publish_active_ ? "ACTIVE" : "INACTIVE");
  }

  void performSeamlessSwitch()
  {
    // Seamless switching without stopping trajectory publishing
    switching_in_progress_ = true;
    
    TrajectoryType new_type = determineTrajectoryType(current_lap_);
    
    if (new_type != current_trajectory_type_ && has_raceline_trajectory_) {
      // Switch trajectory type immediately
      current_trajectory_type_ = new_type;
      has_switched_to_raceline_ = true;
      
      RCLCPP_INFO(get_logger(), "SEAMLESS SWITCH: Lap 0->1 transition to RACELINE trajectory (no interruption)");
      RCLCPP_INFO(get_logger(), "Trajectory publishing continues without pause");
    } else {
      RCLCPP_INFO(get_logger(), "SEAMLESS SWITCH: Lap 0->1 transition, continuing with PITLANE trajectory");
    }
    
    switching_in_progress_ = false;
  }

  TrajectoryType determineTrajectoryType(int lap)
  {
    if (lap == 0) {
      return TrajectoryType::PITLANE;
    } else if (has_raceline_trajectory_) {
      return TrajectoryType::RACELINE;
    } else {
      return TrajectoryType::PITLANE;
    }
  }

  Trajectory getCurrentTrajectory()
  {
    Trajectory current_trajectory;
    
    if (current_trajectory_type_ == TrajectoryType::RACELINE && has_raceline_trajectory_) {
      current_trajectory = raceline_trajectory_;
    } else {
      current_trajectory = pitlane_trajectory_;
    }
    
    // Always update timestamp for fresh data
    current_trajectory.header.stamp = this->now();
    current_trajectory.header.frame_id = "map";
    
    return current_trajectory;
  }
  
  void publish_trajectory()
  {
    if (!publish_active_) {
      return;  // Safety check (though should always be active)
    }
    
    Trajectory current_trajectory = getCurrentTrajectory();
    
    if (current_trajectory.points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No trajectory points to publish");
      return;
    }
    
    // Ensure fresh timestamp for each publish
    current_trajectory.header.stamp = this->now();
    current_trajectory.header.frame_id = "map";
    
    pub_->publish(current_trajectory);
    
    // Reduced logging frequency for performance
    std::string type_name = (current_trajectory_type_ == TrajectoryType::RACELINE) ? "RACELINE" : "PITLANE";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000 /*ms*/, 
      "Publishing %s trajectory continuously @ 20Hz with %zu points (Lap %d, Switch: %s)", 
      type_name.c_str(), current_trajectory.points.size(), current_lap_,
      switching_in_progress_ ? "IN_PROGRESS" : "STABLE");
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
  Trajectory pitlane_trajectory_;
  Trajectory raceline_trajectory_;
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
  
  // Control flags for seamless operation
  bool has_switched_to_raceline_;
  bool switching_in_progress_;
  bool publish_active_;  // Always true for continuous publishing
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CSVToTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
