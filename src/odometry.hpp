#pragma once

#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>
#include <gazebo/physics/Model.hh>
#include <memory>
#include <vector>
#include "vehicle.hpp"

namespace gazebo_ros_four_wheel_steering
{

using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;

class FourWheelSteeringOdometry
{
private:
  FourWheelSteeringVehicle vehicle_;

public:
  explicit FourWheelSteeringOdometry(FourWheelSteeringVehicle vehicle);

  FourWheelSteeringStamped::UniquePtr compute(gazebo::physics::ModelPtr,
    std::vector<gazebo::physics::JointPtr>);
};

}  // namespace gazebo_ros_four_wheel_steering
