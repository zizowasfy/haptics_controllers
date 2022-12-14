// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <haptics_controllers/cartesian_pose_haptics_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace haptics_controllers {

bool CartesianPoseHapticsController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseHapticsController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseHapticsController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseHapticsController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseHapticsController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseHapticsController: Exception getting state handle: " << e.what());
    return false;
  }

  haptics_pose_sub = node_handle.subscribe(
    "haptics_pose", 100, &CartesianPoseHapticsController::hapticsPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void CartesianPoseHapticsController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  std::cout << "Starting: " << initial_pose_[14] << std::endl;
}

void CartesianPoseHapticsController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  if (update_){
    // std::lock_guard<std::mutex> haptics_pose_mutex_lock(haptics_pose_mutex_);

    double radius = 0.10;
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 3.0 * elapsed_time_.toSec()));
    double delta_x = radius * std::sin(angle);
    double delta_z = radius * (std::cos(angle) - 1);

    current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    std::cout << "Update: " << initial_pose_[14] << std::endl;
    std::cout << "current_pose_: " << current_pose_[14] << std::endl;

    std::array<double, 16> new_pose = initial_pose_;
    // new_pose[12] -= delta_x;
    new_pose[14] -= delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
    std::cout << delta_z << std::endl;

    // if (std::abs((initial_pose_[14]+radius) - (current_pose_[14])) < 0.001) {
    //   update_ = false;
    //   std::cout << "Desired Pose REACHED !! " << std::endl;
    //   }

    if (elapsed_time_.toSec() > 5) {update_ = false;}

    

    // std::array<double, 16> new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    // new_pose[14] += haptics_pose_cmd;
    // cartesian_pose_handle_->setCommand(new_pose);


    // current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

    // std::array<double, 16> new_pose = initial_pose_;
    // double m = ((initial_pose_[14]+haptics_deltapose_goal) - (initial_pose_[14])) / (1.0);

    // // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
    // // double delta_z = haptics_deltapose_goal * std::sin(angle);


    // if (std::abs((initial_pose_[14]+haptics_deltapose_goal) - (current_pose_[14])) > 0.001){
    // // if (elapsed_time_.toSec() < 5){
    //   haptics_deltapose_step = m * elapsed_time_.toSec(); // + initial_pose_[14];  // Y = mX + c
    //   new_pose[14] += haptics_deltapose_step;
    //   cartesian_pose_handle_->setCommand(new_pose);
    //   std::cout << " ALMOST THERE !" << std::endl;
    //   std::cout << "haptics_deltapose_step: " << haptics_deltapose_step << std::endl;
    //   std::cout << "current_pose_: " << current_pose_[14] << std::endl;

    //   // ros::Duration(0.5).sleep();
    // }
    // else {
    //   // initial_pose_ = new_pose;
    //   std::cout << "Desired Pose REACHED !! " << std::endl;
    //   update_ = false;
    // }   
 
  }

}

void CartesianPoseHapticsController::hapticsPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  // std::lock_guard<std::mutex> haptics_pose_mutex_lock(haptics_pose_mutex_);
  haptics_pose_cmd = msg->pose.position.z/1000;
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(haptics_controllers::CartesianPoseHapticsController,
                       controller_interface::ControllerBase)
