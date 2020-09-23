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
#include <limits>

#include "gazebo_ros_four_wheel_steering/plugin.hpp"
#include "vehicle.hpp"
#include "controller.hpp"
#include "odometry.hpp"

namespace gazebo_plugins
{
using ignition::math::Vector3d;
using ignition::math::Vector2d;
using gazebo::physics::JointController;
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

  /// Extracts radius of a cylinder or sphere collision shape
  /// \param[in] _coll Pointer to collision
  /// \return If the collision shape is valid, return radius
  /// \return If the collision shape is invalid, return 0
  bool CollisionRadius(const gazebo::physics::CollisionPtr & _coll, double * radius);

  /// Infers the wheel radius from the links attached to the wheel joints
  /// \param[out] _radius Output radius in meters
  /// \return whether the radius can be inferred
  bool InferWheelRadius(double * _radius);

  /// Update joint controller targets
  /// \param[in] dt Delta time in seconds
  void UpdateControllers(double);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<FourWheelSteeringStamped>::SharedPtr cmd_sub_;

  /// Publisher of drive odometry
  rclcpp::Publisher<FourWheelSteeringStamped>::SharedPtr odom_pub_;

  std::unique_ptr<FourWheelSteeringOdometry> odometry_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  FourWheelSteeringVehicle vehicle_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Command in order of reception
  std::deque<FourWheelSteeringStamped> commands_;

  /// Last received drive command
  FourWheelSteeringStamped last_cmd_;

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
  std::string state_frame_id_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  std::vector<std::string> joint_scoped_names_;

  /// PID control parameters
  std::vector<gazebo::common::PID> joint_pids_;

  /// Effort controllers for joints, indices match `JointIdentifier` enu
  std::vector<std::unique_ptr<JointController>> joint_controllers_;

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

static bool IsVelocityJoint(JointIdentifier i)
{
  return i <= REAR_LEFT_MOTOR;
}

static double GetVelocityJointDefaultGain(gazebo::physics::JointPtr joint)
{
  return joint->GetVelocityLimit(0) * joint->GetEffortLimit(0);
}

static double GetPositionJointDefaultGain(gazebo::physics::JointPtr joint)
{
  return joint->UpperLimit(0) * joint->GetEffortLimit(0);
}

static gazebo::sensors::NoisePtr GetSpeedNoiseModel(sdf::ElementPtr _root)
{
  auto speed_sensing = _root->GetElement("speed_sensing");
  if (nullptr == speed_sensing) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
  auto speed_noise_element = speed_sensing->GetElement("noise");
  if (nullptr == speed_noise_element) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
  return gazebo::sensors::NoiseFactory::NewNoiseModel(speed_noise_element);
}

static gazebo::sensors::NoisePtr GetSteeringAngleNoiseModel(sdf::ElementPtr _root)
{
  auto steering_angle_sensing = _root->GetElement("steering_angle_sensing");
  if (nullptr == steering_angle_sensing) {
    return gazebo::sensors::NoisePtr(new gazebo::sensors::Noise(gazebo::sensors::Noise::NONE));
  }
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
  impl_->joints_.resize(6);
  impl_->joint_scoped_names_.resize(6);
  impl_->joint_pids_.resize(6);
  impl_->joint_controllers_.resize(6);

  std::map<JointIdentifier, std::string> joint_names =
  {{FRONT_RIGHT_MOTOR, "front_right_motor"},
    {FRONT_LEFT_MOTOR, "front_left_motor"},
    {REAR_RIGHT_MOTOR, "rear_right_motor"},
    {REAR_LEFT_MOTOR, "rear_left_motor"},
    {FRONT_STEERING, "front_steering"},
    {REAR_STEERING, "rear_steering"},
  };

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
    impl_->joint_scoped_names_[id] = impl_->joints_[id]->GetScopedName();
  }

  for (auto i :
    {FRONT_RIGHT_MOTOR, FRONT_LEFT_MOTOR, REAR_RIGHT_MOTOR, REAR_LEFT_MOTOR, FRONT_STEERING,
      REAR_STEERING})
  {
    auto id = (JointIdentifier)i;
    auto joint_name = joint_names[id];
    auto default_gain =
      IsVelocityJoint(i) ? GetVelocityJointDefaultGain(impl_->joints_[i]) :
      GetPositionJointDefaultGain(
      impl_->joints_[i]);
    if (ignition::math::v4::isnan(default_gain)) {
      default_gain = 1.0;
    }
    auto steering_default_pid = Vector3d(default_gain, 0.0, 0.0);
    auto steering_default_range = Vector2d(-default_gain, default_gain);

    auto pid = _sdf->Get(joint_name + "_pid_gain", steering_default_pid).first;
    auto i_range = _sdf->Get(joint_name + "_i_range", steering_default_range).first;
    impl_->joint_pids_[id].Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

    impl_->joint_controllers_[id] = std::make_unique<JointController>(_model);
    impl_->joint_controllers_[id]->AddJoint(impl_->joints_[id]);

    if (IsVelocityJoint(id)) {
      impl_->joint_controllers_[id]->SetVelocityPID(
        impl_->joint_scoped_names_[id],
        impl_->joint_pids_[id]);

    } else {
      impl_->joint_controllers_[id]->SetPositionPID(
        impl_->joint_scoped_names_[id],
        impl_->joint_pids_[id]);
    }

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Gains [p %.2f, i %.2f, d %.2f] and i_range [%.2f,%.2f] on %s", pid.X(),
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

  auto steering_gear_transmission_ratio_param = _sdf->Get("steering_gear_transmission_ratio", 0.0);
  if (!steering_gear_transmission_ratio_param.second) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "steering_gear_transmission_ratio parameter required, FourWheelSteering cannot be initialized.");
    impl_->ros_node_.reset();
    return;
  }
  impl_->vehicle_.steering_gear_transmission_ratio = steering_gear_transmission_ratio_param.first;

  // Compute wheel_base, front wheel separation, and rear wheel separation
  // first compute the positions of the 4 wheel centers
  // again assumes wheel link is child of joint and has only one collision
  int collision_id = 0;
  auto front_right_center_pos = impl_->joints_[FRONT_RIGHT_MOTOR]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  auto front_left_center_pos = impl_->joints_[FRONT_LEFT_MOTOR]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  auto rear_right_center_pos = impl_->joints_[REAR_RIGHT_MOTOR]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();
  auto rear_left_center_pos = impl_->joints_[REAR_LEFT_MOTOR]->
    GetChild()->GetCollision(collision_id)->WorldPose().Pos();

  auto distance = front_left_center_pos - front_right_center_pos;
  impl_->vehicle_.wheel_track = distance.Length();

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Wheel track: %.2f", impl_->vehicle_.wheel_track);

  auto front_left_axle_center_pos =
    _model->GetLink("front_left_axle")->GetCollision(collision_id)->WorldPose().Pos();
  impl_->vehicle_.distance_steering_to_wheel =
    (front_left_center_pos - front_left_axle_center_pos).Length();

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Distance between wheel and steering joint: %.2f", impl_->vehicle_.distance_steering_to_wheel);

  auto rod = _model->GetJoint("front_right_steering_rod");
  if (nullptr == rod) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Joint [front_right_steering_rod] not found, FourWheelSteering cannot be initialized.");
    impl_->ros_node_.reset();
    return;
  }
  if (rod->GetType() == gazebo::physics::Joint::HINGE_JOINT) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Joint [front_right_steering_rod] is not revolute, FourWheelSteering cannot be initialized.");
    impl_->ros_node_.reset();
    return;
  }
  auto rod_length =
    rod->GetParent()->WorldPose().Pos().Distance(rod->GetChild()->WorldPose().Pos());
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Length of steering rod: %.2f", rod_length);

  auto socket = _model->GetJoint("front_right_steering_connector");
  if (nullptr == socket) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Joint [front_right_steering_connector] not found, FourWheelSteering cannot be initialized.");
    impl_->ros_node_.reset();
    return;
  }
  auto socket_distance = socket->GetParent()->WorldPose().Pos().Distance(
    socket->GetChild()->WorldPose().Pos());
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Distance steering gear to rod connection: %.2f", socket_distance);

  auto front_steering_link = _model->GetLink("front_steering_gear");
  auto rear_steering_link = _model->GetLink("rear_steering_gear");
  auto steering_gear_distance = front_steering_link->WorldPose().Pos().Distance(
    rear_steering_link->WorldPose().Pos());
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Steering gear distance: %.2f", steering_gear_distance);

  auto front_wheel_link = _model->GetLink("front_left_wheel");
  auto rear_wheel_link = _model->GetLink("rear_left_wheel");
  auto front_wheel_x = std::fabs(front_wheel_link->WorldPose().Pos().X());
  auto rear_wheel_x = std::fabs(rear_wheel_link->WorldPose().Pos().X());
  if (std::abs(front_wheel_x - rear_wheel_x) > 0.01) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Axle distance to origin is not equal: %.2f offset front to rear",
      front_wheel_x - rear_wheel_x);
  }

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
    std::bind(&GazeboRosFourWheelSteeringPrivate::OnCmd, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Subscribed to [%s]", impl_->cmd_sub_->get_topic_name());

  impl_->odometry_ = std::make_unique<FourWheelSteeringOdometry>(impl_->vehicle_);
  impl_->odom_pub_ = impl_->ros_node_->create_publisher<FourWheelSteeringStamped>("odom_4ws", 1);

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Publishing [%s]", impl_->odom_pub_->get_topic_name());

  if (_sdf->Get("publish_pid", true).first) {
    for (const auto & joint_name : joint_names) {
      auto name = joint_name.second;
      auto pub =
        impl_->ros_node_->create_publisher<control_msgs::msg::PidState>(
        "pid/" + name,
        rclcpp::QoS(1));

      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing [%s]", pub->get_topic_name());
      impl_->pid_publishers_.push_back(pub);
    }
  }

  impl_->state_frame_id_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosFourWheelSteeringPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosFourWheelSteering::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
  impl_->last_cmd_ = FourWheelSteeringStamped();
  impl_->commands_.clear();
}

void GazeboRosFourWheelSteeringPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> lock(lock_);
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update >= update_period_) {
    UpdateControllers(seconds_since_last_update);
    last_update_time_ = _info.simTime;
    for (auto & controller : joint_controllers_) {
      controller->Update();
    }
    auto msg = odometry_->compute(model_, joints_);
    msg->header.frame_id = state_frame_id_;
    msg->header.stamp = ros_node_->now();
    msg->data.speed = speed_sensing_noise_->Apply(msg->data.speed);
    msg->data.front_steering_angle = steering_angle_sensing_noise_->Apply(
      msg->data.front_steering_angle);
    msg->data.rear_steering_angle = steering_angle_sensing_noise_->Apply(
      msg->data.rear_steering_angle);
    odom_pub_->publish(std::move(msg));
  }
}

void GazeboRosFourWheelSteeringPrivate::UpdateControllers(double dt)
{
  auto ros_time = ros_node_->now();

  while (!commands_.empty() && (ros_time - commands_.front().header.stamp).seconds() >= latency_) {
    last_cmd_ = commands_.front();
    commands_.pop_front();
  }
  if ((ros_time - last_cmd_.header.stamp).seconds() >= (latency_ + command_timeout_)) {
    // if the latest command is too old, set speed to zero
    last_cmd_.data.speed = 0;
  }

  double errors[6];
  double cmds[6];
  compute_wheel_targets(last_cmd_.data, vehicle_, cmds);

  for (auto wheel_i : {FRONT_RIGHT_MOTOR, FRONT_LEFT_MOTOR, REAR_RIGHT_MOTOR, REAR_LEFT_MOTOR}) {
    // set wheel speed efforts
    if (ignition::math::v4::isnan(cmds[wheel_i])) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "NaN command for joint [%s]", joints_[wheel_i]->GetName().c_str());
      cmds[wheel_i] = 0.0;
    }
    // get wheel speed in rad/s
    auto joint_velocity = joints_[wheel_i]->GetVelocity(0);
    errors[wheel_i] = joint_velocity - cmds[wheel_i];

    if (!joint_controllers_[wheel_i]->SetVelocityTarget(
        joint_scoped_names_[wheel_i],
        cmds[wheel_i]))
    {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Joint [%s] was not found", joint_scoped_names_[wheel_i]);
    }
  }

  for (auto steer_i : {FRONT_STEERING, REAR_STEERING}) {
    auto current_angle = joints_[steer_i]->Position(0);
    errors[steer_i] = current_angle - cmds[steer_i];

    // set wheel speed efforts
    if (ignition::math::v4::isnan(cmds[steer_i])) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "NaN command for joint [%s]", joints_[steer_i]->GetName().c_str());
      cmds[steer_i] = 0.0;
    }

    if (!joint_controllers_[steer_i]->SetPositionTarget(
        joint_scoped_names_[steer_i],
        cmds[steer_i]))
    {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Joint [%s] was not found", joint_scoped_names_[steer_i]);
    }
  }

  if (!pid_publishers_.empty()) {
    for (size_t i = FRONT_RIGHT_MOTOR; i <= REAR_STEERING; i++) {
      auto pid = joint_pids_[i];
      control_msgs::msg::PidState state;
      state.header.frame_id = state_frame_id_;
      state.header.stamp = ros_time;
      state.timestep = rclcpp::Duration::from_seconds(dt);
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
}

void GazeboRosFourWheelSteeringPrivate::OnCmd(FourWheelSteeringStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lock_);
  commands_.push_back(*msg);
}

bool GazeboRosFourWheelSteeringPrivate::CollisionRadius(
  const gazebo::physics::CollisionPtr & _coll, double * radius)
{
  if (!_coll || !(_coll->GetShape())) {
    return false;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    *radius = cyl->GetRadius();
    return true;
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    *radius = sph->GetRadius();
    return true;
  }
  return false;
}

bool GazeboRosFourWheelSteeringPrivate::InferWheelRadius(
  double * radius)
{
  // Update wheel radius for wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  // assumes all wheel of both rear wheels of same radii
  double radii[4];

  for (size_t i = 0; i < 4; i++) {
    const auto wheel = joints_[FRONT_RIGHT_MOTOR + i]->GetChild();
    if (wheel->GetCollisions().size() != 1) {
      return false;
    }
    const auto collision_object = wheel->GetCollisions().at(0);
    double radius;
    if (!CollisionRadius(collision_object, &radius)) {
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
