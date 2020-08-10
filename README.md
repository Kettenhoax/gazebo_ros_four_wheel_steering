# Four wheel steering control plugin for Gazebo

Controls a vehicle with front and rear steering based on `four_wheel_steering_msgs/msg/FourWheelSteering`.
Vehicles are modelled with four steering joints and four wheel joints, which are position- and velocity-controlled via tunable PID controllers.

## Installation

* clone to your ros2 workspace
* clone the eloquent version of four_wheel_steering_msgs from [https://github.com/Kettenhoax/four_wheel_steering_msgs/tree/eloquent](https://github.com/Kettenhoax/four_wheel_steering_msgs/tree/eloquent)
* rosdep install --from-path src -i
* colcon build
* source install/setup.bash

Make sure the package environment is sourced, so the library is on `LD_LIBRARY_PATH` when loading a world.

## Running

To run the test world with Gazebo.

```bash
# 1. build project to generate the test model (gazebo_ros_four_wheel_steering_test_vehicle) on the install path
# 2. run test world, loading this model
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so test/worlds/gazebo_ros_four_wheel_steering.world \
  --ros-args --param publish_rate:=200.0
```

To make sure the command timeout and latency simulation work correctly with ROS:

* set `use_sim_time:=true` for the command publisher
* load the ROS libraries into Gazebo
* set an appropriate `publish_rate` for `/clock`.

The joint names in the following configuration example are the defaults, and those joints are required for the plugin to work.
Wheel radius, wheelbase, track and steering offset are determined from the joint distances and collision geometry in the model.

```xml
<model>
    ...
    <plugin name="gazebo_ros_4ws" filename="libgazebo_ros_four_wheel_steering.so">
        <ros>
          <namespace>vehicle</namespace>
        </ros>

        <front_right_motor>front_right_motor</front_right_motor>
        <front_left_motor>front_left_motor</front_left_motor>
        <rear_right_motor>rear_right_motor</rear_right_motor>
        <rear_left_motor>rear_left_motor</rear_left_motor>

        <front_steering>front_steering</front_steering>
        <rear_steering>front_steering</rear_steering>

        <front_right_motor_pid_gain>400 0 1</front_right_motor_pid_gain>
        <front_left_motor_pid_gain>400 0 1</front_left_motor_pid_gain>
        <rear_right_motor_pid_gain>400 0 1</rear_right_motor_pid_gain>
        <rear_left_motor_pid_gain>400 0 1</rear_left_motor_pid_gain>

        <front_steering_pid_gain>2000 0 2</front_steering_pid_gain>
        <rear_steering_pid_gain>2000 0 2</rear_steering_pid_gain>
      </plugin>
      ...
</model>
```

## License and Copyright

`controller.hpp` and `controller.cpp` are adapted from the BSD-licensed [ros_controllers (https://github.com/ros-controls/ros_controllers)](https://github.com/ros-controls/ros_controllers), with a modified interface to the plugin.
The rest of the repository is Apache-2.0-licensed, with copyright holders determined in the file headers.
Contributions to existing files should be made under the license of that file.

The `package.xml` determines the license for files not otherwise marked, with 2020 copyright holder AIT Austrian Institute of Technology GmbH.
New files should be created under this license.