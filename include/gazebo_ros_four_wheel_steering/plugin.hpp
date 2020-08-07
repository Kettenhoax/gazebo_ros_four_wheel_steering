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

#pragma once

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_plugins
{
class GazeboRosFourWheelSteeringPrivate;

enum JointIdentifier
{
  /// Front right traction motor
  FRONT_RIGHT_MOTOR,

  /// Front left traction motor
  FRONT_LEFT_MOTOR,

  /// Rear right traction motor
  REAR_RIGHT_MOTOR,

  /// Rear left traction motor
  REAR_LEFT_MOTOR,

  /// Front steering
  FRONT_STEERING,

  /// Rear steering
  REAR_STEERING,
};

/// A control drive plugin for robots with two steerable axles.
/// Subscribes to four_wheel_steering_msgs/msg/FourWheelSteering

/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_4ws" filename="libgazebo_ros_four_wheel_steering.so">

      <ros>
        <namespace>demo</namespace>
        <remapping>cmd_vel:=cmd_demo</remapping>
      </ros>

      <update_rate>100.0</update_rate>

      <!-- PID tuning -->
      <front_steering_pid_gain>2 0 1</front_steering_pid_gain>
      <front_steering_i_range>0 0</front_steering_i_range>
      ...

      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  \endcode
*/
class GazeboRosFourWheelSteering : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosFourWheelSteering();

  /// Destructor
  ~GazeboRosFourWheelSteering();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosFourWheelSteeringPrivate> impl_;
};
}  // namespace gazebo_plugins
