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

To run the test world with Gazebo:

```bash
# TODO remove absolute path
xacro model.urdf.xacro xacro_prefix:=/home/zeilingerm/code/ws/sim/src/gazebo_ros_four_wheel_steering -o model.urdf
gz sdf
gazebo --verbose test/worlds/gazebo_ros_four_wheel_steering.world
```

The joint names in the following configuration example are the defaults, and they're required joints for the plugin to work. Wheel radius, wheelbase, track and steering offset are determined from joint distances and collision geometry.

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

## Open issues

The front and rear steering axles currently have two independent left and right steering joints each.
There should be a version of the controller that can interface with a central steering joint for each of the axles. With the independent joints it is tricky to tune the controller, as steering has to counteract the drive traction on the wheel forcing the steering joint to turn inwards.

## License and Copyright

`controller.hpp` and `controller.cpp` are taken from the BSD-licensed [ros_controllers (https://github.com/ros-controls/ros_controllers)](https://github.com/ros-controls/ros_controllers), with a modified interface to the plugin.
The rest of the repository is Apache-2.0-licensed, with copyright holders determined in the file headers.
Contributions to existing files should be made under the license of that file.

The `package.xml` determines the license for files not otherwise marked, with 2020 copyright holder AIT Austrian Institute of Technology GmbH.
New files should be created under this license.