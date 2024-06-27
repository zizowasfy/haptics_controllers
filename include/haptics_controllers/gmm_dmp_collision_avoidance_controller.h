// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

#include <haptics_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <std_msgs/Bool.h>
#include <franka/gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <actionlib/client/simple_action_client.h>

using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;

namespace haptics_controllers {

class GMMDMPCollisionAvoidanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pose_feedback_pub_;

  ros::Publisher gripper_action_pub, gripper_stop_pub, gripper_grasp_pub;
  franka_gripper::MoveActionGoal gripper_goal;
  franka_gripper::StopActionGoal gripper_stop;
  franka_gripper::GraspActionGoal gripper_grasp;

  bool gripper_state = false;
  MoveClient* move_client_ptr;
  StopClient* stop_client_ptr;      
  ros::Subscriber sub_gripper_;
  void hapticsGripperCallback(const std_msgs::Bool msg);

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<haptics_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(haptics_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_, sub_learned_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void learnedPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Collision Avoidance
  ros::Subscriber sub_near_collision_;
  std::mutex collision_mutex;
//   Eigen::Matrix<double, 6, 1> collision_vector;
  Eigen::VectorXd collision_vector{6};
  Eigen::Vector3d robot_collision_point;
  franka::Frame collision_link_frame;
  ros::Publisher debug_pub;
  Eigen::Matrix<double, 7, 1> dq_d_obstacle;

  void nearCollisionCallback(const visualization_msgs::MarkerArray& markerarray);
  // Eigen::Matrix<double, 6, 1> avoidLinkCollision(Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq, Eigen::Map<Eigen::Matrix<double, 6, 7>>& J_o);

  // debugging
  ros::Publisher marker_viz_pub;
  //\ debugging

};

}  // namespace haptics_controllers
