// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <haptics_controllers/gmm_dmp_collision_avoidance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <haptics_controllers/pseudo_inversion.h>


namespace haptics_controllers {

bool GMMDMPCollisionAvoidanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // sub_equilibrium_pose_ = node_handle.subscribe(
  //     "equilibrium_pose", 20, &GMMDMPCollisionAvoidanceController::equilibriumPoseCallback, this,
  //     ros::TransportHints().reliable().tcpNoDelay());      

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &GMMDMPCollisionAvoidanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_learned_pose_ = node_handle.subscribe(
      "/gmm/learned_pose", 20, &GMMDMPCollisionAvoidanceController::learnedPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());      

  sub_gripper_ = node_handle.subscribe(
      "/haptics_Gripper", 20, &GMMDMPCollisionAvoidanceController::hapticsGripperCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_near_collision_ = node_handle.subscribe(
      "/near_collision_points_visualization", 20, &GMMDMPCollisionAvoidanceController::nearCollisionCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("GMMDMPCollisionAvoidanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "GMMDMPCollisionAvoidanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "GMMDMPCollisionAvoidanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "GMMDMPCollisionAvoidanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "GMMDMPCollisionAvoidanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "GMMDMPCollisionAvoidanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "GMMDMPCollisionAvoidanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "GMMDMPCollisionAvoidanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<haptics_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&GMMDMPCollisionAvoidanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // Initializing publishers
  pose_feedback_pub_.init(node_handle, "/robot_ee_cartesianpose",100); // initializing the realtime publisher
  
  gripper_action_pub = node_handle.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal",1);
  gripper_stop_pub = node_handle.advertise<franka_gripper::StopActionGoal>("/franka_gripper/stop/goal",1);
  gripper_grasp_pub = node_handle.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal",1);
  //\ Initializing publishers

  // debugging
  debug_pub = node_handle.advertise<std_msgs::Float32MultiArray>("/debugging_topic", 10);
  //\ debugging

  return true;
}

void GMMDMPCollisionAvoidanceController::starting(const ros::Time& /*time*/) {
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

void GMMDMPCollisionAvoidanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7,7>> mass(mass_array.data());
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
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_obstacle(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian, jacobian_pinv);


  /* Computing the nullspace dq (secondary task); moving away from the collision */

  // std::lock_guard<std::mutex> collision_mutex_lock_guard(collision_mutex);
  // std::array<double, 42> collision_point_jacobian_array =
  //   model_handle_->getZeroJacobian(collision_link_frame);
  // Eigen::Map<Eigen::Matrix<double, 6, 7>> collision_point_jacobian(collision_point_jacobian_array.data());

  // Eigen::MatrixXd pinv_term;
  // // pseudoInverse(collision_point_jacobian.transpose() * (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv), pinv_term);
  // pseudoInverse(collision_point_jacobian * (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian), pinv_term);

  // q_d_nullspace_ = pinv_term * (-collision_vector.col(0).normalized() - collision_point_jacobian * jacobian_pinv * (jacobian * dq));

  // // debugging
  // // std::cout << -collision_vector << std::endl;
  // //\ debugging


  // // Steering Angle --------------------------------------------------------------------------------------------------------
  std::lock_guard<std::mutex> collision_mutex_lock_guard(collision_mutex);

  std::array<double, 42> collision_point_jacobian_array =
                          model_handle_->getZeroJacobian(collision_link_frame);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> J_o(collision_point_jacobian_array.data());
  Eigen::MatrixXd pinv_term;
  pseudoInverse(J_o * (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian), pinv_term);


  Eigen::VectorXd dX_o_rot(6), desired_X_o(6);
  Eigen::Vector3d dX_o, r, O_X_vector;

  O_X_vector << collision_vector.head(3);
  dX_o << (J_o * dq).head(3);
  r = O_X_vector.cross(dX_o);

  dX_o_rot.tail(3) << 0.0, 0.0, 0.0;
  dX_o_rot.head(3) << cos(M_PI/2)*dX_o + sin(M_PI/2)*(r.cross(dX_o)) + (1-cos(M_PI/2))*(r.dot(dX_o))*r; 

  double phi = (dX_o.norm() > 0 && collision_vector.norm() != 0) ? acos( (collision_vector.transpose().dot(dX_o)) / (collision_vector.norm()*(dX_o.norm())) ) : 0.0;

  int gamma = 1e7;
  int beta = 20;
  
  desired_X_o.tail(3) << 0.0, 0.0, 0.0;
  desired_X_o.head(3) << gamma * dX_o_rot * phi * exp(-beta/M_PI * abs(phi));
  
  dq_d_obstacle = pinv_term * ((desired_X_o) - J_o * jacobian_pinv * (jacobian * dq));


  // debugging
  // std::vector<float> debug_vector; 
  // debug_vector.push_back(phi);
  // debug_vector(desired_X_o.data(), desired_X_o.data() + desired_X_o.size());
  // std_msgs::Float32MultiArray debug_msg;
  // debug_msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); //debug_msg.layout.dim[0].size = 3;
  // debug_msg.data = debug_vector;
  // debug_pub.publish(debug_msg);

  // std::cout << "dX_o=\n " << dX_o << std::endl;
  // std::cout << "r=\n " << r << std::endl;
  // std::cout << "dX_o_rot=\n " << dX_o_rot << std::endl;
  // std::cout << "phi=\n " << phi << std::endl;
  std::cout << "desired_X_o=\n " << desired_X_o << std::endl;
  std::cout << "dq_d_obstacle=\n " << dq_d_obstacle << std::endl;
  std::cout << "mass*dq_d_obstacle = \n" << mass * dq_d_obstacle << std::endl;

  // \ debugging

  /*\ Computing the nullspace dq (secondary task); moving away from the collision */
  

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * ((q_d_nullspace_ - q)) -
                        (2.0 * sqrt(nullspace_stiffness_)) * (dq)); // Eigen::MatrixXd::Zero(7,1) // (q_d_nullspace_ - q)
  tau_obstacle << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                    mass * dq_d_obstacle;
  // Desired torque
  tau_d << tau_task + tau_nullspace + tau_obstacle + coriolis;
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
  
  // Publish back the Pose of Franka for position/force feedback (haptic mapping)
  if (pose_feedback_pub_.trylock()){
    pose_feedback_pub_.msg_.pose.position.x = position_d_.x(); // 0.378316809929568 - position_d_.x()
    pose_feedback_pub_.msg_.pose.position.y = position_d_.y(); // 0.56330071906726 - position_d_.y();
    pose_feedback_pub_.msg_.pose.position.z = position_d_.z(); // 0.2377368434391054 - position_d_.z();
    orientation_d_.normalize();
    pose_feedback_pub_.msg_.pose.orientation.x = orientation_d_.x();
    pose_feedback_pub_.msg_.pose.orientation.y = orientation_d_.y();
    pose_feedback_pub_.msg_.pose.orientation.z = orientation_d_.z();
    pose_feedback_pub_.msg_.pose.orientation.w = orientation_d_.w();
    pose_feedback_pub_.unlockAndPublish();
  }
  //\ Publish back the Pose of Franka for position/force feedback (haptic mapping)

}

Eigen::Matrix<double, 7, 1> GMMDMPCollisionAvoidanceController::saturateTorqueRate(
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

void GMMDMPCollisionAvoidanceController::nearCollisionCallback(const visualization_msgs::MarkerArray& markerarray)
{
  std::lock_guard<std::mutex> collision_mutex_lock_guard(collision_mutex);
  // Eigen::Matrix<double, 6, 1> collision_vector;
  collision_vector.tail(3) << 0.0, 0.0, 0.0;
  // vector from robot to <octomap> ( markers[0](environment point) - markers[1](robot point) )
  collision_vector.head(3) << markerarray.markers[0].pose.position.x - markerarray.markers[1].pose.position.x,
                              markerarray.markers[0].pose.position.y - markerarray.markers[1].pose.position.y,
                              markerarray.markers[0].pose.position.z - markerarray.markers[1].pose.position.z ;

  // collision_vector.col(0).normalize();
  // collision_vector.normalize();
  // std::cout << collision_vector << std::endl;

  switch (markerarray.markers[1].ns.back())
  {
  case '0':
    // std::cout << typeid(franka::Frame::kJoint1).name() << std::endl;
    collision_link_frame = franka::Frame::kJoint1;
    break;
  case '1':
    collision_link_frame = franka::Frame::kJoint1;
    break;
  case '2':
    collision_link_frame = franka::Frame::kJoint2;
    break;
  case '3':
    collision_link_frame = franka::Frame::kJoint3;
    break;
  case '4':
    collision_link_frame = franka::Frame::kJoint4;
    break;
  case '5':
    collision_link_frame = franka::Frame::kJoint5;
    break;
  case '6':
    collision_link_frame = franka::Frame::kJoint6;
    break;
  case '7':
    collision_link_frame = franka::Frame::kJoint7;
    break;
  case 'd':
    collision_link_frame = franka::Frame::kFlange;
    break;

  default:
    collision_link_frame = franka::Frame::kEndEffector;
    break;
  }
}

// Eigen::Matrix<double, 6, 1> GMMDMPCollisionAvoidanceController::avoidLinkCollision(Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq, Eigen::Map<Eigen::Matrix<double, 6, 7>>& J_o)
// {
//   Eigen::Matrix<double, 6, 1> dX_o = J_o * dq;
//   Eigen::Matrix<double, 6, 1> r = collision_vector.cross(dX_o);
//   Eigen::Matrix<double, 6, 1> dX_o_rot = cos(M_PI/2)*dX_o + sin(M_PI/2)*(r.cross(dX_o)) + (1-cos(M_PI/2))*(r.dot(dX_o))*r; 

//   double phi = acos( (collision_vector.transpose().dot(dX_o_rot)) / (collision_vector.norm()*(dX_o_rot.norm())) );

//   int gamma = 50;
//   int beta = 20;
  
//   return gamma* dX_o_rot * phi * exp(-beta/M_PI * abs(phi));
// }

void GMMDMPCollisionAvoidanceController::complianceParamCallback(
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

void GMMDMPCollisionAvoidanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void GMMDMPCollisionAvoidanceController::learnedPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);

  if (msg->header.frame_id == "delta")
  {
    position_d_target_ += Eigen::Vector3d{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z}; // incrementing the delta position
  }
  else
  {
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
  }
}

void GMMDMPCollisionAvoidanceController::hapticsGripperCallback(const std_msgs::Bool msg){
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
    gripper_grasp.goal.force = 60.0;
    gripper_grasp.goal.epsilon.inner = 0.03;
    gripper_grasp.goal.epsilon.outer = 0.03;

    gripper_grasp_pub.publish(gripper_grasp);
    ROS_INFO_STREAM("Gripper Closed");
  }
}

}  // namespace haptics_controllers

PLUGINLIB_EXPORT_CLASS(haptics_controllers::GMMDMPCollisionAvoidanceController,
                       controller_interface::ControllerBase)
