#pragma once

#include <memory>
#include <string>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/PID.hh>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

namespace gazebo_ros_four_wheel_steering
{

class JointController
{
public:
  typedef std::unique_ptr<JointController> UniquePtr;

  virtual void update(rclcpp::Time, rclcpp::Duration, double cmd) = 0;

  virtual double state() const = 0;

  virtual double get_error() = 0;

  virtual void set_pid(gazebo::common::PID pid) = 0;
};

class JointControllerFactory
{
public:
  JointController::UniquePtr create(
    gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf,
    rclcpp::Node::SharedPtr _node, std::string name);
};

}  // namespace gazebo_ros_four_wheel_steering
