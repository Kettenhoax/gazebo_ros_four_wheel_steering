# Four wheel steering control plugin for Gazebo

Controls a vehicle with front and rear steering based on `four_wheel_steering_msgs/msg/FourWheelSteering`.

## Installation

* clone to your ros2 workspace
* clone `four_wheel_steering_msgs` from [https://github.com/Kettenhoax/four_wheel_steering_msgs/tree/foxy](https://github.com/Kettenhoax/four_wheel_steering_msgs/tree/foxy)
* rosdep install --from-path src -i
* colcon build
* source install/setup.bash

Make sure the package environment is sourced, so the library is on `LD_LIBRARY_PATH` when spawning a robot into a Gazebo world.

## Running

To run the test world with Gazebo.

```bash
# 1. build project to generate the test model (gazebo_ros_four_wheel_steering_test_vehicle)
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

### Tests

When running tests using `colcon test`, make sure the install folder is sourced, as the test will timeout searching the wrong MODEL_PATH otherwise.

## Model

The model uses a joint with multiple parents for each wheel kingpin.
The kinematic chain for steering mechanics are only modelled in sdf, as part of a `<gazebo>` element in the URDF model.
There are two joint motors for steering, and four traction motors for the wheels.
In URDF, the kingpin joint is only parented to base_link, so the tf tree can be visualized.

If you model your own vehicle, you can use the urdf macros in the urdf subfolder.

## Parameters

The following parameters can be passed as children on the sdf plugin element.

* `steering_gear_transmission_ratio` (required): conversion ratio for wheel steering angle to steering gear angle (gear angle = wheel angle * -steering_gear_transmission_ratio)
* `latency`: seconds to wait before applying commands to the joint motors, useful to simulate real-world latency of vehicle commands
* `command_timeout`: seconds to wait until invalidating the last command and setting motor target velocity to zero, latency will be added implicitly
* `update_rate`: update rate in Hz
* `publish_pid`: enable publishing of PID controller values as ROS topic
* `{joint_name}_pid_gain`: gains in order PID for respective joint motor
* `{joint_name}_i_range`: integral error bounds for respective joint motor
* `{joint_name}`: change the plugins search name for logical joint

Parameters on the geometry of the vehicle are derived from the links and joints on the model.

### Subscriptions

* `cmd_4ws`: drive commands of type `four_wheel_steering_msgs/msg/FourWheelSteeringStamped`

### Publishers

* `odom_4ws`: drive odometry of type `four_wheel_steering_msgs/msg/FourWheelSteeringStamped`
* `pid/{joint_name}`: PID state of type `control_msgs/msg/PidState`, enabled if `publish_pid` is true

### Example

```xml
<model>
    ...
    <plugin name="gazebo_ros_4ws" filename="libgazebo_ros_four_wheel_steering.so">
        <ros>
          <namespace>vehicle</namespace>
        </ros>

        <robot_base_frame>base_footprint</robot_base_frame>
        <steering_gear_transmission_ratio>1.5</steering_gear_transmission_ratio>
        <update_rate>100.0</update_rate>

        <front_right_motor>front_right_motor</front_right_motor>
        <front_left_motor>front_left_motor</front_left_motor>
        <rear_right_motor>rear_right_motor</rear_right_motor>
        <rear_left_motor>rear_left_motor</rear_left_motor>

        <front_steering>front_steering</front_steering>
        <rear_steering>front_steering</rear_steering>

        <front_right_motor_pid_gain>1 0 0</front_right_motor_pid_gain>
        <front_left_motor_pid_gain>1 0 0</front_left_motor_pid_gain>
        <rear_right_motor_pid_gain>1 0 0</rear_right_motor_pid_gain>
        <rear_left_motor_pid_gain>1 0 0</rear_left_motor_pid_gain>

        <front_steering_pid_gain>1 0 0</front_steering_pid_gain>
        <rear_steering_pid_gain>1 0 0</rear_steering_pid_gain>
      </plugin>
      ...
</model>
```

### Advanced

Additional parameters enable simulation of noise on the output odometry as well as latency behaviour.

```xml
<model>
  ...
  <plugin name="gazebo_ros_4ws" filename="libgazebo_ros_four_wheel_steering.so">
    ...
    <!-- wait 100ms before applying a command -->
    <latency>0.1</latency>
    <!-- if the command timestamp is older than 20ms, ignore and stop the vehicle -->
    <command_timeout>0.02</command_timeout>

    <!-- configure noise on the speed and steering angles measured for odom_4ws -->
    <speed_sensing>
        <noise type="gaussian_quantized">
          <stddev>0.005</stddev>
          <!-- quantize speed field of odom_4ws to 0.1 kph -->
          <precision>0.027777778</precision>
        </noise>
    </speed_sensing>
    <steering_angle_sensing>
        <noise type="gaussian_quantized">
          <stddev>0.005</stddev>
          <!-- quantize front and rear steering angle to 1 degree -->
          <precision>0.02</precision>
        </noise>
    </steering_angle_sensing>
    ...
  </plugin>
</model>
```

## License and Copyright

`controller.hpp` and `controller.cpp` are adapted from the BSD-licensed [ros_controllers (https://github.com/ros-controls/ros_controllers)](https://github.com/ros-controls/ros_controllers), with a modified interface to the plugin.
The rest of the repository is Apache-2.0-licensed, with copyright holders determined in the file headers.
Contributions to existing files should be made under the license of that file.

The `package.xml` determines the license for files not otherwise marked, with 2020 copyright holder AIT Austrian Institute of Technology GmbH.
New files should be created under this license.