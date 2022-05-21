// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

class JointVelocityExampleController : public controller_interface::MultiInterfaceController<
                                           franka_hw::FrankaModelInterface,
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  ros::Duration elapsed_time_;

  //added by jyp on 0517
  ros::Publisher pub_jacoian_matrix_;
  ros::Publisher pub_joint_angles_;
  ros::Publisher pub_joint_velocities_;
  ros::Publisher pub_ee_pose;
  ros::Subscriber sub_q_d_;
  Eigen::Matrix<double, 7, 1> saturateJointVelocity(
      const Eigen::Matrix<double, 7, 1>& q_d_this_update_,
      const Eigen::Matrix<double, 7, 1>& q_d_current_executed_
  );
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  bool is_initialized = true;
  int pub_counter;
  const double joint_velocity_max_{1.5};
  const double joint_acceleration_max_{0.5};
  void jointVelocityCommandCallback(const std_msgs::Float64MultiArray& msg);
};

}  // namespace franka_example_controllers
