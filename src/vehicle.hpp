#pragma once

namespace gazebo_plugins
{

struct FourWheelSteeringVehicle
{
  /// Axial distance between the wheels, in meters.
  double wheel_base;

  /// Distance between front and rear axles, in meters.
  double wheel_track;

  /// Radius of rear wheels, in meters.
  double wheel_radius;

  /// Distance between any wheel and its steering joint
  double distance_steering_to_wheel;

  /// Transmission ratio of steering gear to wheel steering angle
  double steering_gear_transmission_ratio;
};

} // namespace gazebo_plugins
