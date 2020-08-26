#pragma once

#include "vehicle.hpp"
#include <gazebo/physics/Model.hh>
#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>

namespace gazebo_plugins
{

using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;

class FourWheelSteeringOdometry
{
private:
  FourWheelSteeringVehicle vehicle_;

public:
  FourWheelSteeringOdometry(FourWheelSteeringVehicle vehicle);

  FourWheelSteeringStamped::UniquePtr compute(gazebo::physics::ModelPtr, std::vector<gazebo::physics::JointPtr>);
};

} // namespace gazebo_plugins
