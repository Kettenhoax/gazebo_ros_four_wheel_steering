#include "joint.hpp"
#include <string>
#include <memory>
#include <utility>
#include <gazebo/physics/JointController.hh>
#include <rclcpp/publisher.hpp>
#include <control_msgs/msg/pid_state.hpp>

namespace gazebo_ros_four_wheel_steering
{
using namespace std; // NOLINT
using gazebo::physics::ModelPtr;
using gazebo::physics::JointPtr;
using gazebo::common::PID;
using control_msgs::msg::PidState;

typedef gazebo::physics::JointController GazeboJointController;

class BaseJointController : public JointController
{
protected:
  /// Controlled joint
  JointPtr joint_;

  /// PID controller for joint_
  unique_ptr<GazeboJointController> joint_controller_;

  PID pid_params_;

  double error_;

public:
  BaseJointController(
    JointPtr j,
    unique_ptr<GazeboJointController> c,
    PID initial_pid)
  : JointController(), joint_(j), joint_controller_(move(c)), error_(0.0)
  {
    joint_controller_->AddJoint(joint_);
    set_pid(initial_pid);
  }

  double get_error() override {
    return error_;
  }

  void set_pid(PID pid) override
  {
    pid_params_ = pid;
    joint_controller_->SetPositionPID(joint_->GetScopedName(), pid);
  }
};

inline bool ends_with(string const & value, string const & ending)
{
  if (ending.size() > value.size()) {return false;}
  return equal(ending.rbegin(), ending.rend(), value.rbegin());
}

class MotorJointController : public BaseJointController
{
public:
  MotorJointController(
    JointPtr j,
    unique_ptr<GazeboJointController> c,
    PID initial_pid)
  : BaseJointController(j, std::move(c), initial_pid) {}

  virtual ~MotorJointController() = default;

  virtual double state() const
  {
    return joint_->GetVelocity(0);
  }

  virtual void update(rclcpp::Time t, rclcpp::Duration dt, double cmd)
  {
    auto current_velocity = joint_->GetVelocity(0);
    error_ = current_velocity - cmd;

    if (ignition::math::isnan(cmd)) {
      throw invalid_argument("NaN command for joint [" + joint_->GetName() + "]");
    }
    if (!joint_controller_->SetVelocityTarget(joint_->GetScopedName(), cmd)) {
      throw invalid_argument(
              "JointController failed to find joint [" + joint_->GetScopedName() + "]");
    }
  }
};

class SteeringJointController : public BaseJointController
{
public:
  SteeringJointController(
    JointPtr j,
    unique_ptr<GazeboJointController> c,
    PID initial_pid)
  : BaseJointController(j, std::move(c), initial_pid) {}

  virtual ~SteeringJointController() = default;

  virtual double state() const
  {
    return joint_->Position(0);
  }

  virtual void update(rclcpp::Time t, rclcpp::Duration dt, double cmd)
  {
    auto current_position = joint_->Position(0);
    error_ = current_position - cmd;

    if (ignition::math::isnan(cmd)) {
      throw invalid_argument("NaN command for joint [" + joint_->GetName() + "]");
    }
    if (!joint_controller_->SetPositionTarget(joint_->GetScopedName(), cmd)) {
      throw invalid_argument(
              "JointController failed to find joint [" + joint_->GetScopedName() + "]");
    }
  }
};

JointController::UniquePtr JointControllerFactory::create(
  ModelPtr _model, sdf::ElementPtr _sdf,
  rclcpp::Node::SharedPtr _node, string name)
{
  // get remapped name from sdf parameters, or use default name
  auto joint = _model->GetJoint(name);
  if (!joint) {
    throw invalid_argument(
            "Joint [" + name + "] not found, FourWheelSteering cannot be initialized.");
  }

  double kp = _node->declare_parameter(name + ".kp", 1.0);
  double ki = _node->declare_parameter(name + ".ki", 0);
  double kd = _node->declare_parameter(name + ".kd", 0);
  auto pid = PID(kp, ki, kd);

  rclcpp::Publisher<PidState>::SharedPtr pid_publisher;

  if (_sdf->Get("publish_pid", true).first) {
    pid_publisher = _node->create_publisher<control_msgs::msg::PidState>(
      "pid/" + name,
      rclcpp::QoS(1));
    RCLCPP_INFO(
      _node->get_logger(),
      "Publishing PID state on [%s]", pid_publisher->get_topic_name());
  }

  // unique_ptr<JointController> controller;
  if (ends_with(name, "motor")) {
    return make_unique<MotorJointController>(joint, pid);
  } else {
    return make_unique<SteeringJointController>(joint, pid);
  }
  // TODO(ZeilingerM) reenable PID publisher
  // if (pid_publisher) {
  //   return make_unique<PIDPublisherJointControllerDecorator>(controller, pid_publisher);
  // }
  // return controller;
}

}  // namespace gazebo_ros_four_wheel_steering
