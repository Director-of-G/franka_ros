// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  ros::Duration(2.0).sleep();

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  // added by jyp on 0517
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  is_initialized = false;
  pub_counter = 0;
  pub_jacoian_matrix_ = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/zero_jacobian", 10);
  pub_joint_angles_ = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/joint_angles", 10);
  pub_joint_velocities_ = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/joint_velocities", 10);
  pub_ee_pose = node_handle.advertise<std_msgs::Float64MultiArray>("/gazebo_sim/ee_pose", 10);
  sub_q_d_ = node_handle.subscribe("/gazebo_sim/joint_velocity_desired", 10, 
      &JointVelocityExampleController::jointVelocityCommandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  is_initialized = true;
}

void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // read robot state
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // publish franka state
  if (pub_counter >= 10) {
    pub_counter = 0;
    std_msgs::Float64MultiArray msg1;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 7; j++) {
            msg1.data.push_back(jacobian(i, j));
        }
    }
    pub_jacoian_matrix_.publish(msg1);

    std_msgs::Float64MultiArray msg2;
    for (int i = 0; i < 7; i++) {
        msg2.data.push_back(q(i));
    }
    pub_joint_angles_.publish(msg2);

    std_msgs::Float64MultiArray msg3;
    for (int i = 0; i < 7; i++) {
        msg3.data.push_back(dq(i));
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
  else {
    pub_counter++;
  }

  ros::Duration time_max(8.0);
  double omega_max = 0.1;
  double cycle = std::floor(
      std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
                         time_max.toSec()));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

  // for (auto joint_handle : velocity_joint_handles_) {
  //   joint_handle.setCommand(omega);
  // }
}

Eigen::Matrix<double, 7, 1> JointVelocityExampleController::saturateJointVelocity(
    const Eigen::Matrix<double, 7, 1>& q_d_this_update_,
    const Eigen::Matrix<double, 7, 1>& q_d_current_executed_) {
  Eigen::Matrix<double, 7, 1> q_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = q_d_this_update_[i] - q_d_current_executed_[i];
    q_d_saturated[i] = 
        q_d_current_executed_[i] + std::max(std::min(difference, joint_acceleration_max_), -joint_acceleration_max_);
    q_d_saturated[i] = std::max(std::min(q_d_saturated[i], joint_velocity_max_), -joint_velocity_max_);
  }
  return q_d_saturated;
}

void JointVelocityExampleController::jointVelocityCommandCallback(const std_msgs::Float64MultiArray& msg)
{
  std::cout << "Enter joint velocity command callback!" << std::endl;
  if (is_initialized == false) {
    return;
  }
  std::cout << "Start to saturate joint velocity command!" << std::endl;
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Matrix<double, 7, 1> joint_velocity_command;
  for (size_t i = 0; i < 7; i++) {
    joint_velocity_command[i] = msg.data.at(i);
  }
  Eigen::Matrix<double, 7, 1> joint_velocity_command_saturated = saturateJointVelocity(joint_velocity_command, dq);
  for (size_t i = 0; i < 7; i++) {
    hardware_interface::JointHandle joint_handle = velocity_joint_handles_[i];
    joint_handle.setCommand(joint_velocity_command_saturated[i]);
    std::cout << "joint: " << i << " set speed: " << joint_velocity_command_saturated[i] << std::endl;
  }
  std::cout << "Set joint velocity successfully!" << std::endl;
  ROS_INFO("Set joint velocity successfully!");
}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
