#pragma once

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
#include <string>
#include <memory>
#include <utility>
#include <limits>
#include <vector>
#include <algorithm>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>

namespace gazebo_ros_four_wheel_steering
{

using namespace std; // NOLINT
using gazebo::physics::JointPtr;

struct JointNames
{
  string front_left_motor;
  string front_right_motor;
  string rear_left_motor;
  string rear_right_motor;

  JointNames()
  : front_left_motor(), front_right_motor(), rear_left_motor(), rear_right_motor()
  {

  }
};

struct FourWheelSteeringVehicle
{
  /// Axial distance between the wheels, in meters.
  double wheel_base;

  /// Distance between front and rear axles, in meters.
  double wheel_track;

  /// Radius of rear wheels, in meters.
  double wheel_radius;

  /// Distance between any wheel and its steering joint
  double steering_gear_distance;

  /// Distance between any wheel and its steering joint
  double distance_steering_to_wheel;

  /// Transmission ratio of steering gear to wheel steering angle
  double steering_gear_transmission_ratio;

  double socket_distance;

  /// Length of the steering rod
  double steering_rod_length;

  JointNames joint_names;

  FourWheelSteeringVehicle()
  : wheel_base(0), wheel_track(0), wheel_radius(0), distance_steering_to_wheel(0),
    steering_gear_transmission_ratio(0), joint_names()
  {

  }
};

class vehicle_parameter_error : public std::exception
{
private:
  std::string message_;

public:
  explicit vehicle_parameter_error(std::string message);
  const char * what() const noexcept override
  {
    return message_.c_str();
  }
};

vehicle_parameter_error::vehicle_parameter_error(std::string message)
: message_(message)
{

}

// Replace std::unique_ptr with std::optional in C++17
unique_ptr<double> get_collision_radius(const gazebo::physics::CollisionPtr & _coll)
{
  if (!_coll || !(_coll->GetShape())) {
    return nullptr;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    return make_unique<double>(cyl->GetRadius());
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    return make_unique<double>(sph->GetRadius());
  }
  return nullptr;
}

template<typename T>
struct skip
{
  T & t;
  std::size_t n;
  skip(T & v, std::size_t s)
  : t(v), n(s) {}
  auto begin()->decltype(std::begin(t))
  {
    return std::next(std::begin(t), n);
  }
  auto end()->decltype(std::end(t))
  {
    return std::end(t);
  }
};

// Replace std::unique_ptr with std::optional in C++17
unique_ptr<double> get_wheel_radius(gazebo::physics::ModelPtr _model, JointNames _names)
{
  // Update wheel radius for wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  // assumes all wheel of both rear wheels of same radii
  vector<double> radii;

  for (auto joint_name : {_names.front_left_motor, _names.front_right_motor, _names.rear_left_motor,
      _names.rear_right_motor})
  {
    const auto wheel = _model->GetJoint(joint_name)->GetChild();
    if (wheel->GetCollisions().size() != 1) {
      return nullptr;
    }
    auto radius = get_collision_radius(wheel->GetCollisions().at(0));
    if (!radius) {
      return nullptr;
    }
    radii.push_back(*radius);
  }
  for (auto radius : skip<decltype(radii)>(radii, 1)) {
    if (fabs(radius - radii[0]) > radius * std::numeric_limits<double>::epsilon()) {
      return nullptr;
    }
  }
  return make_unique<double>(radii[0]);
}

ignition::math::Vector3d get_joint_collision_pos(gazebo::physics::ModelPtr _model, string joint_name)
{
  auto link = _model->GetJoint(joint_name)->GetChild();
  if (link->GetCollisions().size() > 0) {
    throw vehicle_parameter_error(
            "Joint " + joint_name + " has more than one child link collision objects");
  }
  unsigned int collision_id = 0;
  return link->GetCollision(collision_id)->WorldPose().Pos();
}

/// Distance between connected links of a joint
double joint_distance(JointPtr joint)
{
  return joint->GetParent()->WorldPose().Pos().Distance(
    joint->GetChild()->WorldPose().Pos());
}

FourWheelSteeringVehicle get_vehicle_parameters(
  gazebo::physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  FourWheelSteeringVehicle result;

  result.joint_names.front_left_motor =
    _sdf->Get<string>("front_left_motor", "front_left_motor").first;
  result.joint_names.front_right_motor =
    _sdf->Get<string>("front_right_motor", "front_right_motor").first;
  result.joint_names.rear_left_motor =
    _sdf->Get<string>("rear_left_motor", "rear_left_motor").first;
  result.joint_names.rear_right_motor =
    _sdf->Get<string>("rear_right_motor", "rear_right_motor").first;

  auto wheel_radius_param = _sdf->Get("wheel_radius", 0.0);
  if (wheel_radius_param.second) {
    result.wheel_radius = wheel_radius_param.first;
  } else {
    auto radius = get_wheel_radius(_model, result.joint_names);
    if (!radius) {
      result.wheel_radius = *radius;
    } else {
      throw vehicle_parameter_error("Wheel radius cannot be determined");
    }
  }

  auto steering_gear_transmission_ratio_param = _sdf->Get("steering_gear_transmission_ratio", 0.0);
  if (!steering_gear_transmission_ratio_param.second) {
    throw vehicle_parameter_error("steering_gear_transmission_ratio parameter required");
  }
  result.steering_gear_transmission_ratio = steering_gear_transmission_ratio_param.first;

  // Compute wheel_base, front wheel separation, and rear wheel separation
  // first compute the positions of the 4 wheel centers
  // again assumes wheel link is child of motor joint and has a single collision object
  auto front_left_center_pos = get_joint_collision_pos(_model, result.joint_names.front_left_motor);
  auto front_right_center_pos =
    get_joint_collision_pos(_model, result.joint_names.front_right_motor);
  auto rear_left_center_pos = get_joint_collision_pos(_model, result.joint_names.rear_left_motor);
  auto rear_right_center_pos = get_joint_collision_pos(_model, result.joint_names.rear_right_motor);

  auto distance = front_left_center_pos - front_right_center_pos;
  result.wheel_track = distance.Length();

  unsigned int collision_id = 0;
  auto front_left_axle_center_pos =
    _model->GetLink("front_left_axle")->GetCollision(collision_id)->WorldPose().Pos();
  result.distance_steering_to_wheel = front_left_center_pos.Distance(front_left_axle_center_pos);

  auto rod = _model->GetJoint("front_right_steering_rod");
  if (nullptr == rod) {
    throw vehicle_parameter_error("Failed to find joint [front_right_steering_rod]");
  }
  // TODO(ZeilingerM) condition mismatches with message
  if (rod->GetType() == gazebo::physics::Joint::HINGE_JOINT) {
    throw vehicle_parameter_error("Joint [front_right_steering_rod] must be revolute");
  }
  result.steering_rod_length = joint_distance(rod);

  auto socket = _model->GetJoint("front_right_steering_connector");
  if (nullptr == socket) {
    throw vehicle_parameter_error("Failed to find joint [front_right_steering_connector]");
  }
  result.socket_distance = joint_distance(socket);

  auto front_steering_link = _model->GetLink("front_steering_gear");
  auto rear_steering_link = _model->GetLink("rear_steering_gear");
  result.steering_gear_distance = front_steering_link->WorldPose().Pos().Distance(
    rear_steering_link->WorldPose().Pos());

  auto front_wheel_link = _model->GetLink("front_left_wheel");
  auto rear_wheel_link = _model->GetLink("rear_left_wheel");
  auto front_wheel_x = std::fabs(front_wheel_link->WorldPose().Pos().X());
  auto rear_wheel_x = std::fabs(rear_wheel_link->WorldPose().Pos().X());
  if (std::abs(front_wheel_x - rear_wheel_x) > 0.01) {
    auto delta = front_wheel_x - rear_wheel_x;
    throw vehicle_parameter_error(
            "Axle distance to origin differs between front and rear by " + to_string(
              delta));
  }

  // to compute wheelbase, first position of axle centers are computed
  auto front_axle_pos = (front_left_center_pos + front_right_center_pos) / 2;
  auto rear_axle_pos = (rear_left_center_pos + rear_right_center_pos) / 2;
  // then the wheelbase is the distance between the axle centers
  distance = front_axle_pos - rear_axle_pos;
  result.wheel_base = distance.Length();

  return result;
}

}  // namespace gazebo_ros_four_wheel_steering
