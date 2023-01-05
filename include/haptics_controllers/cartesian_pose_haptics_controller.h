// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>
// #include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <geometry_msgs/PoseStamped.h>

namespace haptics_controllers {

class CartesianPoseHapticsController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_{};

  // std::mutex haptics_pose_mutex_;
  bool update_ = true;
  double haptics_deltapose_goal = 0.02;
  double haptics_deltapose_step = 0.0;
  double haptics_pose_cmd = 0.0;
  std::array<double, 16> current_pose_{}; ;
  // franka::RobotState robot_state;
  ros::Subscriber haptics_pose_sub;
  void hapticsPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace haptics_controllers
