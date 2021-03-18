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

#include <gazebo/physics/Joint.hh>
#include <vector>
#include <memory>
#include "gazebo_ros_four_wheel_steering/plugin.hpp"
#include "odometry.hpp"

namespace gazebo_ros_four_wheel_steering
{

using gazebo::physics::ModelPtr;
using gazebo::physics::JointPtr;

FourWheelSteeringOdometry::FourWheelSteeringOdometry(FourWheelSteeringVehicle vehicle)
: vehicle_(vehicle) {}

double common_angle(double left, double right)
{
  return atan(2 * tan(left) * tan(right) / (tan(left) + tan(right)));
}

FourWheelSteeringStamped::UniquePtr FourWheelSteeringOdometry::compute(ModelPtr model)
{
  auto odom = std::make_unique<FourWheelSteeringStamped>();

  JointPtr front_left_kingpin = model->GetJoint("front_left_kingpin");
  JointPtr front_right_kingpin = model->GetJoint("front_right_kingpin");
  JointPtr rear_left_kingpin = model->GetJoint("rear_left_kingpin");
  JointPtr rear_right_kingpin = model->GetJoint("rear_right_kingpin");

  double front_angle = common_angle(
    front_left_kingpin->Position(0), front_right_kingpin->Position(0));
  double rear_angle = common_angle(
    rear_left_kingpin->Position(0), rear_right_kingpin->Position(0));
  odom->data.front_steering_angle = front_angle;
  odom->data.rear_steering_angle = rear_angle;

  auto rads_fl = joints[FRONT_LEFT_MOTOR]->GetVelocity(0);
  auto rads_fr = joints[FRONT_RIGHT_MOTOR]->GetVelocity(0);
  auto rads_rl = joints[REAR_LEFT_MOTOR]->GetVelocity(0);
  auto rads_rr = joints[REAR_RIGHT_MOTOR]->GetVelocity(0);
  auto rads_avg = (rads_fl + rads_fr + rads_rl + rads_rr) / 4.0;

  odom->data.speed = vehicle_.wheel_radius * rads_avg;
  return odom;
}

}  // namespace gazebo_ros_four_wheel_steering
