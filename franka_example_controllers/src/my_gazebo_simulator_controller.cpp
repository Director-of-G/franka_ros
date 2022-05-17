// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_gazebo_simulator_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>
#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

bool MyGazeboSimulatorController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  // Add by jyp on 0514
  pub_jacoian_matrix_ = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/zero_jacobian", 10);
  pub_joint_angles_ = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/joint_angles", 10);
  pub_joint_velocities_ = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/joint_velocities", 10);
  pub_ee_pose = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/ee_pose", 10);
  

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("MyGazeboSimulatorController: Could not read parameter arm_id");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MyGazeboSimulatorController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MyGazeboSimulatorController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MyGazeboSimulatorController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MyGazeboSimulatorController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }
  
  return true;
}

void MyGazeboSimulatorController::starting(const ros::Time& /*time*/) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
}

void MyGazeboSimulatorController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // Add by jyp on 0514
  franka::RobotState initial_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_angles(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_velocities(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> zero_jacobian(jacobian_array.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // Publish ros messages
  std_msgs::Float64MultiArray msg1;
  for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 7; j++) {
          msg1.data.push_back(zero_jacobian(i, j));
      }
  }
  pub_jacoian_matrix_.publish(msg1);

  std_msgs::Float64MultiArray msg2;
  for (int i = 0; i < 7; i++) {
      msg2.data.push_back(joint_angles(i));
  }
  pub_joint_angles_.publish(msg2);

  std_msgs::Float64MultiArray msg3;
  for (int i = 0; i < 7; i++) {
      msg3.data.push_back(joint_velocities(i));
  }
  pub_joint_velocities_.publish(msg3);

  std_msgs::Float64MultiArray msg4;
  for (int i = 0; i < 3; i++) {
      msg4.data.push_back(position(i));
  }
  Eigen::Matrix<double, 4, 1> quat = orientation.coeffs();
  for (int i = 0; i < 4; i++) {
      msg4.data.push_back(quat(i));
  }
  pub_ee_pose.publish(msg4);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyGazeboSimulatorController,
                       controller_interface::ControllerBase)
