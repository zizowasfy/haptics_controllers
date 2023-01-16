// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <haptics_controllers/cartesian_impedance_haptics_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <haptics_controllers/pseudo_inversion.h>


namespace haptics_controllers {

bool CartesianImpedanceHapticsController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // sub_equilibrium_pose_ = node_handle.subscribe(
  //     "equilibrium_pose", 20, &CartesianImpedanceHapticsController::equilibriumPoseCallback, this,
  //     ros::TransportHints().reliable().tcpNoDelay());      

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/touch_teleop_deltapose", 20, &CartesianImpedanceHapticsController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());      

  sub_gripper_ = node_handle.subscribe(
      "/haptics_Gripper", 20, &CartesianImpedanceHapticsController::hapticsGripperCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // MoveClient move_client_("/franka_gripper/move", true);      
  // StopClient stop_client_("/franka_gripper/stop", true);      
  // move_client_ptr = &move_client_;
  // stop_client_ptr = &stop_client_;

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceHapticsController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceHapticsController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceHapticsController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceHapticsController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceHapticsController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceHapticsController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceHapticsController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceHapticsController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<haptics_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceHapticsController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  pose_feedback_pub_.init(node_handle, "/robot_ee_cartesianpose",100); // initializing the realtime publisher
  
  gripper_action_pub = node_handle.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal",1);
  gripper_stop_pub = node_handle.advertise<franka_gripper::StopActionGoal>("/franka_gripper/stop/goal",1);
  gripper_grasp_pub = node_handle.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal",1);

  return true;
}

void CartesianImpedanceHapticsController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  gripper_goal.goal.width = 0.0;
}

void CartesianImpedanceHapticsController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);

  // position_d_target_ = initial_transform.translation();

  // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  position_d_ = filter_params_ * (position_d_target_) + (1.0 - filter_params_) * position_d_;  // adding delta position
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  if (pose_feedback_pub_.trylock()){
    orientation_d_.normalize();
    pose_feedback_pub_.msg_.pose.orientation.x = orientation_d_.x();
    pose_feedback_pub_.msg_.pose.orientation.y = orientation_d_.y();
    pose_feedback_pub_.msg_.pose.orientation.z = orientation_d_.z();
    pose_feedback_pub_.msg_.pose.orientation.w = orientation_d_.w();
    pose_feedback_pub_.unlockAndPublish();
  }

  // SENDING on the ACTIONGOAL topic to open/close the gripper
  // franka_gripper::MoveGoal gripper_goal;
  // if (gripper_state && gripper_goal.goal.width == 0.0) // True = Opens
  // {
  //   // gripper_stop_pub.publish(gripper_stop);
  //   gripper_goal.goal.speed = 0.1;
  //   gripper_goal.goal.width = 0.07;
  //   gripper_action_pub.publish(gripper_goal);
    
  //   ROS_INFO_STREAM("Gripper Opened");
  // }
  // else if (!gripper_state && gripper_goal.goal.width == 0.07)
  // {  // gripper_goal.goal.width = 0.0;
  //   // gripper_action_pub.publish(gripper_goal);
  //   gripper_goal.goal.width = 0.07;
  //   gripper_grasp.goal.width = 0.0;
  //   gripper_grasp.goal.speed = 0.1;
  //   gripper_grasp.goal.force = 1.0;
  //   gripper_grasp.goal.epsilon.inner = 0.02;
  //   gripper_grasp.goal.epsilon.outer = 0.02;

  //   gripper_grasp_pub.publish(gripper_grasp);
  //   ROS_INFO_STREAM("Gripper Closed");
  // }
  // \ SENDING on the ACTIONGOAL topic to open/close the gripper

  //  // TRYING to USE the ACTIONLIB to open/close the gripper
  // move_client_("/franka_gripper/move", true);
  // stop_client_("/franka_gripper/stop", true);
  // ROS_INFO_STREAM(gripper_state);

  // if (gripper_state){
  //     // Open gripper
  //   ROS_INFO_STREAM("gripper Opened");
  //   franka_gripper::MoveGoal move_goal;
  //   move_goal.speed = 0.1;
  //   move_goal.width = 0.01;
  //   move_client_ptr->sendGoal(move_goal);
  //   std::cout << "GOAL SENT" << std::endl;
  //   if (move_client_ptr->waitForResult(ros::Duration(5.0))) {
  //     gripper_state = false;
  //   }
  // }
  //   } else {
  //   ROS_INFO_STREAM("gripper Closed");
  //         // Close gripper
  //   franka_gripper::MoveGoal move_goal;
  //   move_goal.speed = 0.1;
  //   move_goal.width = 0.05;
  //   move_client_ptr->sendGoal(move_goal);
  //   // if (move_client_ptr->waitForResult(ros::Duration(5.0))) {
  //   //   gripper_state = true;
  //   // } else {
  //   //   ROS_ERROR("haptics_gripper_node: MoveAction was not successful.");
  //   //   stop_client_ptr->sendGoal(franka_gripper::StopGoal());
  //   // }
  // }
  // // \ TRYING to USE the ACTIONLIB to open/close the gripper

}

Eigen::Matrix<double, 7, 1> CartesianImpedanceHapticsController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceHapticsController::complianceParamCallback(
    haptics_controllers::compliance_paramConfig &config,
    uint32_t /*level*/)
{
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceHapticsController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  // delta_position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z; // receiving the delta position

  position_d_target_ += Eigen::Vector3d{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z}; // incrementing the delta position
  // position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

  orientation_d_target_ = orientation_d_target_ * orientation_d_;   // Quaternionmultiplying the delta quaternion by 
                                                                   // current quaternion to get the new orientation quaternion

  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void CartesianImpedanceHapticsController::hapticsGripperCallback(const std_msgs::Bool msg){
  gripper_state = msg.data;
  if (gripper_state)
  {
    // gripper_stop_pub.publish(gripper_stop);
    gripper_goal.goal.speed = 0.1;
    gripper_goal.goal.width = 0.07;
    gripper_action_pub.publish(gripper_goal);
    
    ROS_INFO_STREAM("Gripper Opened");
  }
  else
  {
    gripper_grasp.goal.width = 0.0;
    gripper_grasp.goal.speed = 0.1;
    gripper_grasp.goal.force = 1.0;
    gripper_grasp.goal.epsilon.inner = 0.02;
    gripper_grasp.goal.epsilon.outer = 0.02;

    gripper_grasp_pub.publish(gripper_grasp);
    ROS_INFO_STREAM("Gripper Closed");
  }
}

}  // namespace haptics_controllers

PLUGINLIB_EXPORT_CLASS(haptics_controllers::CartesianImpedanceHapticsController,
                       controller_interface::ControllerBase)
