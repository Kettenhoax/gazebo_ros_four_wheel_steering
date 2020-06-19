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

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <memory>
#include <string>
#include <vector>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <four_wheel_steering_msgs/msg/four_wheel_steering.hpp>
#include <control_msgs/msg/pid_state.hpp>

#include "gazebo_ros_four_wheel_steering/plugin.hpp"
#include "controller.hpp"

namespace gazebo_plugins
{
using ignition::math::Vector3d;
using ignition::math::Vector2d;

class GazeboRosFourWheelSteeringPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a drive command is received.
  /// \param[in] _msg FourWheelSteering command message.
  void OnCmd(four_wheel_steering_msgs::msg::FourWheelSteering::SharedPtr _msg);

  /// Extracts radius of a cylinder or sphere collision shape
  /// \param[in] _coll Pointer to collision
  /// \return If the collision shape is valid, return radius
  /// \return If the collision shape is invalid, return 0
  double CollisionRadius(const gazebo::physics::CollisionPtr & _coll);

  /// Infers the wheel radius from the links attached to the wheel joints
  /// \param[out] _radius Output radius in meters
  /// \return whether the radius can be inferred
  bool InferWheelRadius(double * _radius);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<four_wheel_steering_msgs::msg::FourWheelSteering>::SharedPtr cmd_sub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  FourWheelSteeringVehicle vehicle_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Last received command, initialized with zero-speed and zero-angle
  four_wheel_steering_msgs::msg::FourWheelSteering last_cmd_;

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// PID control for left steering control
  std::vector<gazebo::common::PID> joint_pids_;

  /// ROS publishers for PID states per joint
  std::vector<rclcpp::Publisher<control_msgs::msg::PidState>::SharedPtr> pid_publishers_;
};

GazeboRosFourWheelSteering::GazeboRosFourWheelSteering()
: impl_(std::make_unique<GazeboRosFourWheelSteeringPrivate>())
{
}

GazeboRosFourWheelSteering::~GazeboRosFourWheelSteering()
{
}

void GazeboRosFourWheelSteering::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();
  auto physicsEngine = world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  impl_->last_cmd_.speed = 0;
  impl_->last_cmd_.front_steering_angle = 0;
  impl_->last_cmd_.rear_steering_angle = 0;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  impl_->joints_.resize(8);
  impl_->joint_pids_.resize(8);

  std::map<JointIdentifier, std::string> joint_names =
  {{FRONT_RIGHT, "front_right_wheel"},
    {FRONT_LEFT, "front_left_wheel"},
    {REAR_RIGHT, "rear_right_wheel"},
    {REAR_LEFT, "rear_left_wheel"},
    {FRONT_RIGHT_STEERING, "front_right_steering"},
    {FRONT_LEFT_STEERING, "front_left_steering"},
    {REAR_RIGHT_STEERING, "rear_right_steering"},
    {REAR_LEFT_STEERING, "rear_left_steering"}};

  for (const auto & joint_name : joint_names) {
    auto id = joint_name.first;
    auto name = joint_name.second;
    // get remapped name from sdf parameters, or use default name
    auto remapped_name = _sdf->Get<std::string>(name, name).first;
    impl_->joints_[id] = _model->GetJoint(remapped_name);
    if (!impl_->joints_[id]) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(),
        "Joint [%s] not found, FourWheelSteering cannot be initialized.", name.c_str());
      impl_->ros_node_.reset();
      return;
    }
  }

  for (size_t i = FRONT_RIGHT; i < FRONT_RIGHT + 4; i++) {
    auto id = (JointIdentifier)i;
    auto joint_name = joint_names[id];
    auto default_gain = impl_->joints_[i]->GetVelocityLimit(0) *
      impl_->joints_[i]->GetEffortLimit(0);
    if (ignition::math::v4::isnan(default_gain)) {
      default_gain = 1.0;
    }
    auto speed_default_pid = Vector3d(default_gain, 0.0, 0.0);
    auto speed_default_range = Vector2d(-default_gain, default_gain);

    auto pid = _sdf->Get(joint_name + "_pid_gain", speed_default_pid).first;
    auto i_range = _sdf->Get(joint_name + "_i_range", speed_default_range).first;
    impl_->joint_pids_[id].Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Gains [p %.2f, i %.2f, d %.2f] and integral bounds [%.2f,%.2f] on %s", pid.X(),
      pid.Y(), pid.Z(), i_range.X(), i_range.Y(), joint_name.c_str());
  }

  for (size_t i = FRONT_RIGHT_STEERING; i < FRONT_RIGHT_STEERING + 4; i++) {
    auto id = (JointIdentifier)i;
    auto joint_name = joint_names[id];
    auto default_gain = impl_->joints_[i]->UpperLimit(0) * impl_->joints_[i]->GetEffortLimit(0);
    if (ignition::math::v4::isnan(default_gain)) {
      default_gain = 1.0;
    }
    auto steering_default_pid = Vector3d(default_gain, 0.0, 0.0);
    auto steering_default_range = Vector2d(-default_gain, default_gain);

    auto pid = _sdf->Get(joint_name + "_pid_gain", steering_default_pid).first;
    auto i_range = _sdf->Get(joint_name + "_i_range", steering_default_range).first;
    impl_->joint_pids_[id].Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Gains [p %.2f, i %.2f, d %.2f] and integral bounds [%.2f,%.2f] on %s", pid.X(),
      pid.Y(), pid.Z(), i_range.X(), i_range.Y(), joint_name.c_str());
  }

  auto wheel_radius_param = _sdf->Get("wheel_radius", 0.0);
  if (wheel_radius_param.second) {
    impl_->vehicle_.wheel_radius = wheel_radius_param.first;
  } else if (impl_->InferWheelRadius(&impl_->vehicle_.wheel_radius)) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Wheel radius: %.2f", impl_->vehicle_.wheel_radius);
  } else {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Wheel radius not given and cannot be inferred, FourWheelSteering cannot be initialized.");
    impl_->ros_node_.reset();
    return;
  }

  // Compute wheel_base, front wheel separation, and rear wheel separation
  // first compute the positions of the 4 wheel centers
  // again assumes wheel link is child of joint and has only one collision
  int collision_id = 0;
  auto front_right_center_pos = impl_->joints_[FRONT_RIGHT]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  auto front_left_center_pos = impl_->joints_[FRONT_LEFT]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  auto rear_right_center_pos = impl_->joints_[REAR_RIGHT]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  auto rear_left_center_pos = impl_->joints_[REAR_LEFT]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();

  auto distance = front_left_center_pos - front_right_center_pos;
  impl_->vehicle_.wheel_track = distance.Length();

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Wheel track: %.2f", impl_->vehicle_.wheel_track);

  auto front_left_steer_center_pos = impl_->joints_[FRONT_LEFT_STEERING]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  impl_->vehicle_.distance_steering_to_wheel =
    (front_left_center_pos - front_left_steer_center_pos).Length();

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Distance between wheel and steering joint: %.2f", impl_->vehicle_.distance_steering_to_wheel);

  // to compute wheelbase, first position of axle centers are computed
  auto front_axle_pos = (front_left_center_pos + front_right_center_pos) / 2;
  auto rear_axle_pos = (rear_left_center_pos + rear_right_center_pos) / 2;
  // then the wheelbase is the distance between the axle centers
  distance = front_axle_pos - rear_axle_pos;
  impl_->vehicle_.wheel_base = distance.Length();

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Wheel base: %.2f", impl_->vehicle_.wheel_base);

  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->cmd_sub_ =
    impl_->ros_node_->create_subscription<four_wheel_steering_msgs::msg::FourWheelSteering>(
    "cmd_4ws", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&GazeboRosFourWheelSteeringPrivate::OnCmd, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Subscribed to [%s]", impl_->cmd_sub_->get_topic_name());

  if (_sdf->Get("publish_pid", true).first) {
    for (const auto & joint_name : joint_names) {
      auto name = joint_name.second;
      auto pub =
        impl_->ros_node_->create_publisher<control_msgs::msg::PidState>(
        "pid/" + name,
        rclcpp::QoS(
          rclcpp::KeepLast(
            1)));
      impl_->pid_publishers_.push_back(pub);
    }
  }

  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosFourWheelSteeringPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosFourWheelSteering::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
  impl_->last_cmd_.speed = 0;
  impl_->last_cmd_.front_steering_angle = 0;
  impl_->last_cmd_.rear_steering_angle = 0;
}

void GazeboRosFourWheelSteeringPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> lock(lock_);
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }
  auto ros_time = ros_node_->now();

  std::vector<double> errors;
  errors.resize(8);

  double cmds[8];
  compute_wheel_targets(last_cmd_, vehicle_, cmds);

  for (size_t wheel_i = FRONT_RIGHT; wheel_i < FRONT_RIGHT + 4; wheel_i++) {
    // get wheel speed in rad/s
    auto joint_velocity = joints_[wheel_i]->GetVelocity(0);
    auto wheelspeed_error = joint_velocity - cmds[wheel_i];
    errors[wheel_i] = wheelspeed_error;
    double wheelspeed_cmd =
      joint_pids_[wheel_i].Update(wheelspeed_error, seconds_since_last_update);
    // set wheel speed efforts
    if (ignition::math::v4::isnan(wheelspeed_cmd)) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "PID computed NaN command for [%lu]", wheel_i);
    } else {
      joints_[wheel_i]->SetForce(0, wheelspeed_cmd);
    }
  }

  for (size_t steer_i = FRONT_RIGHT_STEERING; steer_i < FRONT_RIGHT_STEERING + 4;
    steer_i++)
  {
    auto current_angle = joints_[steer_i]->Position(0);
    auto angle_error = current_angle - cmds[steer_i];
    errors[steer_i] = angle_error;
    auto steering_cmd = joint_pids_[steer_i].Update(angle_error, seconds_since_last_update);
    joints_[steer_i]->SetForce(0, steering_cmd);
  }

  if (!pid_publishers_.empty()) {
    for (size_t i = FRONT_RIGHT; i < 8; i++) {
      auto pid = joint_pids_[i];
      control_msgs::msg::PidState state;
      state.header.frame_id = robot_base_frame_;
      state.header.stamp = ros_time;
      state.timestep = rclcpp::Duration::from_seconds(seconds_since_last_update);
      state.error = errors[i];
      state.error_dot = NAN;
      pid.GetErrors(state.p_error, state.i_error, state.d_error);
      state.p_term = pid.GetPGain();
      state.i_term = pid.GetIGain();
      state.d_term = pid.GetDGain();
      state.i_max = pid.GetIMax();
      state.i_min = pid.GetIMin();
      state.output = pid.GetCmd();
      pid_publishers_[i]->publish(state);
    }
  }

  last_update_time_ = _info.simTime;
}

void GazeboRosFourWheelSteeringPrivate::OnCmd(
  const four_wheel_steering_msgs::msg::FourWheelSteering::SharedPtr msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  last_cmd_ = *msg;
}

double GazeboRosFourWheelSteeringPrivate::CollisionRadius(
  const gazebo::physics::CollisionPtr & _coll)
{
  if (!_coll || !(_coll->GetShape())) {
    return 0;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    return cyl->GetRadius();
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

bool GazeboRosFourWheelSteeringPrivate::InferWheelRadius(double * radius)
{
  // Update wheel radius for wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  // assumes all wheel of both rear wheels of same radii
  unsigned int id = 0;
  double radii[4];

  for (size_t i = 0; i < 4; i++) {
    const auto & joint = joints_[FRONT_RIGHT + i];
    const auto & collision_object = joint->GetChild()->GetCollision(id);
    auto radius = CollisionRadius(collision_object);
    if (radius < 0.01) {
      return false;
    }
    radii[i] = radius;
  }
  for (size_t i = 1; i < 4; i++) {
    if (fabs(radii[i] - radii[0]) > radii[0] * std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  *radius = radii[0];
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosFourWheelSteering)
}  // namespace gazebo_plugins
