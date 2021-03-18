// Copyright 2020 AIT Austrian Institute of Technology GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>
#include <control_msgs/msg/pid_state.hpp>

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Noise.hh>
#include <sdf/sdf.hh>

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <utility>

#include "gazebo_ros_four_wheel_steering/plugin.hpp"
#include "controller.hpp"
#include "joint.hpp"
// #include "odometry.hpp"
#include "vehicle.hpp"

namespace gazebo_ros_four_wheel_steering
{
using namespace std; // NOLINT
using ignition::math::Vector3d;
using ignition::math::Vector2d;
using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;

class GazeboRosFourWheelSteeringPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a drive command is received.
  /// \param[in] _msg FourWheelSteering command message.
  void OnCmd(FourWheelSteeringStamped::SharedPtr _msg);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<FourWheelSteeringStamped>::SharedPtr cmd_sub_;

  /// Publisher of drive odometry
  rclcpp::Publisher<FourWheelSteeringStamped>::SharedPtr odom_pub_;

  // unique_ptr<FourWheelSteeringOdometry> odometry_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  FourWheelSteeringVehicle vehicle_;

  /// Protect variables accessed on callbacks.
  mutex lock_;

  /// Command in order of reception
  deque<FourWheelSteeringStamped> command_msgs_;

  /// Last received drive command
  FourWheelSteeringStamped last_cmd_;

  JointController::UniquePtr front_left_controller_;

  JointController::UniquePtr front_right_controller_;

  JointController::UniquePtr rear_left_controller_;

  JointController::UniquePtr rear_right_controller_;

  /// Noise applied to speed odometry
  gazebo::sensors::NoisePtr speed_sensing_noise_;

  /// Noise applied to front and rear steering angle odometry
  gazebo::sensors::NoisePtr steering_angle_sensing_noise_;

  /// Update period in seconds.
  double update_period_;

  /// Command timeout in seconds
  double command_timeout_;

  /// Artificial latency to application of a received command in seconds
  double latency_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Robot base frame ID
  string state_frame_id_;
};

GazeboRosFourWheelSteering::GazeboRosFourWheelSteering()
: impl_(make_unique<GazeboRosFourWheelSteeringPrivate>())
{
}

GazeboRosFourWheelSteering::~GazeboRosFourWheelSteering()
{
}

// static double GetVelocityJointDefaultGain(gazebo::physics::JointPtr joint)
// {
//   return joint->GetVelocityLimit(0) * joint->GetEffortLimit(0);
// }

// static double GetPositionJointDefaultGain(gazebo::physics::JointPtr joint)
// {
//   return joint->UpperLimit(0) * joint->GetEffortLimit(0);
// }

static gazebo::sensors::NoisePtr GetSpeedNoiseModel(sdf::ElementPtr _root)
{
  if (!_root->HasElement("speed_sensing")) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
  auto speed_sensing = _root->GetElement("speed_sensing");
  auto speed_noise_element = speed_sensing->GetElement("noise");
  if (nullptr == speed_noise_element) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
  return gazebo::sensors::NoiseFactory::NewNoiseModel(speed_noise_element);
}

static gazebo::sensors::NoisePtr GetSteeringAngleNoiseModel(sdf::ElementPtr _root)
{
  if (!_root->HasElement("steering_angle_sensing")) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
  auto steering_angle_sensing = _root->GetElement("steering_angle_sensing");
  auto steering_angle_noise_element = steering_angle_sensing->GetElement("noise");
  if (nullptr == steering_angle_noise_element) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
  return gazebo::sensors::NoiseFactory::NewNoiseModel(steering_angle_noise_element);
}

void GazeboRosFourWheelSteering::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();
  auto physicsEngine = world->Physics();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  try {
    impl_->vehicle_ = get_vehicle_parameters(_model, _sdf);
  } catch (const vehicle_parameter_error & e) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "Failed to get vehicle parameters: %s", e.what());
    throw e;
  }

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Wheel base: %.2f", impl_->vehicle_.wheel_base);
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Wheel track: %.2f", impl_->vehicle_.wheel_track);
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Wheel radius: %.2f", impl_->vehicle_.wheel_radius);
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Steering gear distance: %.2f", impl_->vehicle_.steering_gear_distance);
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Distance between wheel and steering joint: %.2f", impl_->vehicle_.distance_steering_to_wheel);
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Distance steering gear to rod connection: %.2f", impl_->vehicle_.socket_distance);
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Length of steering rod: %.2f", impl_->vehicle_.steering_rod_length);

  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->command_timeout_ = _sdf->Get<double>("command_timeout", impl_->update_period_ * 2).first;
  impl_->latency_ = _sdf->Get<double>("latency", 0.0).first;

  impl_->speed_sensing_noise_ = GetSpeedNoiseModel(_sdf);
  if (impl_->speed_sensing_noise_->GetNoiseType() != gazebo::sensors::Noise::NONE) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Applying noise model to speed sensing");
  }
  impl_->steering_angle_sensing_noise_ = GetSteeringAngleNoiseModel(_sdf);
  if (impl_->steering_angle_sensing_noise_->GetNoiseType() != gazebo::sensors::Noise::NONE) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Applying noise model to steering angle sensing");
  }

  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->last_cmd_ = FourWheelSteeringStamped();
  impl_->cmd_sub_ =
    impl_->ros_node_->create_subscription<FourWheelSteeringStamped>(
    "cmd_4ws", rclcpp::QoS(1),
    bind(&GazeboRosFourWheelSteeringPrivate::OnCmd, impl_.get(), placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Subscribed to [%s]", impl_->cmd_sub_->get_topic_name());

  // impl_->odometry_ = make_unique<FourWheelSteeringOdometry>(impl_->vehicle_);
  // impl_->odom_pub_ = impl_->ros_node_->create_publisher<FourWheelSteeringStamped>("odom_4ws", 1);
  // RCLCPP_INFO(
  //   impl_->ros_node_->get_logger(),
  //   "Publishing [%s]", impl_->odom_pub_->get_topic_name());

  impl_->state_frame_id_ = _sdf->Get<string>("robot_base_frame", "base_footprint").first;

  JointControllerFactory factory;
  impl_->front_left_controller_ =
    factory.create(
      _model, _sdf, impl_->ros_node_,
      impl_->vehicle_.joint_names.front_left_motor);
  impl_->front_right_controller_ =
    factory.create(
      _model, _sdf, impl_->ros_node_,
      impl_->vehicle_.joint_names.front_left_motor);
  impl_->rear_left_controller_ =
    factory.create(
      _model, _sdf, impl_->ros_node_,
      impl_->vehicle_.joint_names.front_left_motor);
  impl_->rear_right_controller_ =
    factory.create(
      _model, _sdf, impl_->ros_node_,
      impl_->vehicle_.joint_names.front_left_motor);

  // TODO(ZeilingerM) reenable
  // RCLCPP_INFO(
  //   impl_->ros_node_->get_logger(),
  //   "Gains [p %.2f, i %.2f, d %.2f] and i_range [%.2f,%.2f] on %s", pid.X(),
  //   pid.Y(), pid.Z(), i_range.X(), i_range.Y(), joint_name.c_str());

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    bind(&GazeboRosFourWheelSteeringPrivate::OnUpdate, impl_.get(), placeholders::_1));
}

void GazeboRosFourWheelSteering::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
  impl_->last_cmd_ = FourWheelSteeringStamped();
  impl_->command_msgs_.clear();
}

void GazeboRosFourWheelSteeringPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  lock_guard<mutex> lock(lock_);
  double dt = (_info.simTime - last_update_time_).Double();
  if (dt >= update_period_) {
    auto ros_time = ros_node_->now();
    while (!command_msgs_.empty() &&
      (ros_time - command_msgs_.front().header.stamp).seconds() >= latency_)
    {
      last_cmd_ = command_msgs_.front();
      command_msgs_.pop_front();
    }
    if ((ros_time - last_cmd_.header.stamp).seconds() >= (latency_ + command_timeout_)) {
      // if the latest command is too old, set speed to zero
      last_cmd_.data.speed = 0;
    }

    auto cmd = compute_four_wheel_steering_command(last_cmd_.data, vehicle_);
    auto rdt = rclcpp::Duration::from_seconds(dt);
    front_left_controller_->update(ros_time, rdt, cmd.front_left_radps);
    front_right_controller_->update(ros_time, rdt, cmd.front_right_radps);
    rear_left_controller_->update(ros_time, rdt, cmd.rear_left_radps);
    rear_right_controller_->update(ros_time, rdt, cmd.rear_right_radps);

    // auto msg = odometry_->compute(model_, joints_);
    // msg->header.frame_id = state_frame_id_;
    // msg->header.stamp = ros_node_->now();
    // msg->data.speed = speed_sensing_noise_->Apply(msg->data.speed);
    // msg->data.front_steering_angle = steering_angle_sensing_noise_->Apply(
    //   msg->data.front_steering_angle);
    // msg->data.rear_steering_angle = steering_angle_sensing_noise_->Apply(
    //   msg->data.rear_steering_angle);
    // odom_pub_->publish(move(msg));

    last_update_time_ = _info.simTime;
  }
}

void GazeboRosFourWheelSteeringPrivate::OnCmd(FourWheelSteeringStamped::SharedPtr msg)
{
  lock_guard<mutex> lock(lock_);
  command_msgs_.push_back(*msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosFourWheelSteering)
}  // namespace gazebo_ros_four_wheel_steering
