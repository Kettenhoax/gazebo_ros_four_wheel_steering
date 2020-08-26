#include "odometry.hpp"
#include "gazebo_ros_four_wheel_steering/plugin.hpp"
#include <gazebo/physics/Joint.hh>

namespace gazebo_plugins
{

FourWheelSteeringOdometry::FourWheelSteeringOdometry(FourWheelSteeringVehicle vehicle)
: vehicle_(vehicle) {}

double common_angle(double left, double right)
{
  return atan(2 * tan(left) * tan(right) / (tan(left) + tan(right)));
}

FourWheelSteeringStamped::UniquePtr FourWheelSteeringOdometry::compute(
  gazebo::physics::ModelPtr model,
  std::vector<gazebo::physics::JointPtr> joints)
{
  auto odom = std::make_unique<FourWheelSteeringStamped>();

  gazebo::physics::JointPtr front_left_kingpin = model->GetJoint("front_left_kingpin");
  gazebo::physics::JointPtr front_right_kingpin = model->GetJoint("front_right_kingpin");

  gazebo::physics::JointPtr rear_left_kingpin = model->GetJoint("rear_left_kingpin");
  gazebo::physics::JointPtr rear_right_kingpin = model->GetJoint("rear_right_kingpin");

  double front_angle = common_angle(
    front_left_kingpin->Position(0), front_right_kingpin->Position(0));
  double rear_angle = common_angle(
    rear_left_kingpin->Position(0), rear_right_kingpin->Position(0));
  odom->data.front_steering_angle = front_angle;
  odom->data.rear_steering_angle = rear_angle;

  double denom = (tan(front_angle) - tan(rear_angle)) / vehicle_.wheel_base;
  double ft = cos(front_angle) / denom;
  double rt = cos(rear_angle) / denom;

  auto rads_fl = joints[FRONT_LEFT_MOTOR]->GetVelocity(0);
  auto rads_fr = joints[FRONT_RIGHT_MOTOR]->GetVelocity(0);
  auto rads_rl = joints[REAR_LEFT_MOTOR]->GetVelocity(0);
  auto rads_rr = joints[REAR_RIGHT_MOTOR]->GetVelocity(0);

  double steering_track = vehicle_.wheel_track - 2 * vehicle_.distance_steering_to_wheel;
  auto v_front =
    sqrt((rads_fl * rads_fl + rads_fr * rads_fr) / (2 + std::pow(steering_track * ft / 2, 2)));
  auto v_rear =
    sqrt((rads_rl * rads_rl + rads_rr * rads_rr) / (2 + std::pow(steering_track * rt / 2, 2)));

  odom->data.speed = vehicle_.wheel_radius * (v_front + v_rear) / 2;
  return odom;
}

} // namespace gazebo_plugins
