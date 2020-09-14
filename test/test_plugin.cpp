// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include <gazebo/common/Time.hh>
#include <gazebo/test/ServerFixture.hh>
#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// floating-point tolerance for test comparisons
#define tol 10e-1

using namespace std::literals::chrono_literals; // NOLINT
using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;

class GazeboRosFourWheelSteeringTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosFourWheelSteeringTest, DriveStraight)
{
  this->Load("test/worlds/gazebo_ros_four_wheel_steering.world", true);

  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  auto vehicle = world->ModelByName("gazebo_ros_four_wheel_steering_test_vehicle");
  ASSERT_NE(nullptr, vehicle);

  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_four_wheel_steering_test", options);
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = node->create_publisher<FourWheelSteeringStamped>("cmd_4ws", rclcpp::QoS(1));

  // Step a bit for model to settle
  world->Step(100);
  executor.spin_once(100ms);

  // Check model state
  EXPECT_NEAR(0.0, vehicle->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldPose().Rot().Yaw(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldLinearVel().X(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldAngularVel().Z(), tol);

  auto msg = FourWheelSteeringStamped();
  msg.header.frame_id = "base_link";
  msg.data.speed = 1.0;
  msg.data.front_steering_angle = 0.0;
  msg.data.rear_steering_angle = 0.0;

  // Process simulation in 10ms-increments for 1 second
  size_t steps{100};
  for (size_t step = 0; step < steps; ++step) {
    msg.header.stamp = node->get_clock()->now();
    pub->publish(msg);
    world->Step(10);
    executor.spin_once(10ms);
  }

  // Check that the vehicle accelerated to at least a velocity of 1 m/s
  EXPECT_LT(0.0, vehicle->WorldPose().Pos().X());
  EXPECT_NEAR(1.0, vehicle->WorldLinearVel().X(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldAngularVel().Z(), tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
