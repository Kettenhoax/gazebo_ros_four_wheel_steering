/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <cmath>
#include <gazebo_ros_four_wheel_steering/plugin.hpp>
#include "controller.hpp"

namespace gazebo_plugins
{

using four_wheel_steering_msgs::msg::FourWheelSteering;

void compute_wheel_targets(
  const FourWheelSteering & cmd_4ws,
  const FourWheelSteeringVehicle & vehicle, double cmds[6])
{
// Compute steering angles
  const double tan_front_steering = tan(cmd_4ws.front_steering_angle);
  const double tan_rear_steering = tan(cmd_4ws.rear_steering_angle);

  const double steering_track = vehicle.wheel_track - 2 * vehicle.distance_steering_to_wheel;
  const double steering_diff = steering_track * (tan_front_steering - tan_rear_steering) / 2.0;

  auto wb = vehicle.wheel_base;

  double front_left_angle = 0.0;
  double front_right_angle = 0.0;
  double rear_left_angle = 0.0;
  double rear_right_angle = 0.0;

  if (fabs(vehicle.wheel_base - fabs(steering_diff)) > 0.001) {
    // compute optimal angles resulting from ackermann steering, to determine optimal torque on motors
    front_left_angle = atan(wb * tan_front_steering / (wb - steering_diff));
    front_right_angle = atan(wb * tan_front_steering / (wb + steering_diff));
    rear_left_angle = atan(wb * tan_rear_steering / (wb - steering_diff));
    rear_right_angle = atan(wb * tan_rear_steering / (wb + steering_diff));

    // convert from wheel angle to steering gear angle
    cmds[FRONT_STEERING] = -1 * cmd_4ws.front_steering_angle;
    cmds[REAR_STEERING] = -1 * cmd_4ws.rear_steering_angle;
  }

  // Compute wheels velocities
  if (fabs(cmd_4ws.speed) > 0.001) {
    //Virutal front and rear wheelbase
    // distance between the projection of the CIR on the wheelbase and the front axle
    double l_front = 0;
    if (fabs(tan(front_left_angle) - tan(front_right_angle)) >
      0.01)
    {
      l_front = tan(front_right_angle) *
        tan(front_left_angle) * steering_track /
        (tan(front_left_angle) - tan(front_right_angle));
    }
    // distance between the projection of the CIR on the wheelbase and the rear axle
    double l_rear = 0;
    if (fabs(tan(rear_left_angle) - tan(rear_right_angle)) > 0.01) {
      l_rear = tan(rear_right_angle) * tan(rear_left_angle) * steering_track /
        (tan(rear_left_angle) - tan(rear_right_angle));
    }

    const double angular_speed_cmd = cmd_4ws.speed * (tan_front_steering - tan_rear_steering) /
      wb;
    const double vel_steering_offset = (angular_speed_cmd * vehicle.distance_steering_to_wheel) /
      vehicle.wheel_radius;
    const double sign = copysign(1.0, cmd_4ws.speed);

    cmds[FRONT_LEFT_MOTOR] = sign * std::hypot(
      (cmd_4ws.speed - angular_speed_cmd * steering_track / 2),
      (l_front * angular_speed_cmd)) / vehicle.wheel_radius -
      vel_steering_offset;
    cmds[FRONT_RIGHT_MOTOR] = sign * std::hypot(
      (cmd_4ws.speed + angular_speed_cmd * steering_track / 2),
      (l_front * angular_speed_cmd)) / vehicle.wheel_radius +
      vel_steering_offset;
    cmds[REAR_LEFT_MOTOR] = sign * std::hypot(
      (cmd_4ws.speed - angular_speed_cmd * steering_track / 2),
      (l_rear * angular_speed_cmd)) / vehicle.wheel_radius -
      vel_steering_offset;
    cmds[REAR_RIGHT_MOTOR] = sign * std::hypot(
      (cmd_4ws.speed + angular_speed_cmd * steering_track / 2),
      (l_rear * angular_speed_cmd)) / vehicle.wheel_radius +
      vel_steering_offset;
  }
}

} // namespace gazebo_plugins
