# Offboard Example

In order to comprehend this example, make sure you have read the [Offboard Mode](https://docs.px4.io/master/en/flight_modes/offboard.html) article in the PX4 documentation. A detailed documentation on this example can be found [here](https://docs.px4.io/v1.12/en/ros/ros2_offboard_control.html).

To better understand the mode changes in the code, have a look at the [published examples](../publishers) of the repository.

For the example start any robot, e.g.

```
user@ubuntu:~/PX4-Autopilot$ make px4_sitl_rtps gazebo_rover
```

In another terminal and in a new terminal the bridge agent

```
user@ubuntu:~/colcon-ws$ micrortps_agent -t UDP
```


# Run the example

> **_NOTE:_** In order to make this example work, you need [QGC](https://dev.px4.io/v1.11_noredirect/en/qgc/) running, as the offboard mode requires this as fail safe in PX4.

You can specify xyz coordinates in the local [NED](https://docs.px4.io/master/en/contribute/notation.html#acronyms) coordinates to define you target position like this

```
user@ubuntu:~/colcon-ws$ ros2 run px4_ros_com_offboard_example offboard_control --ros-args \
    -p pos_x_m:=10.0 \
    -p pos_y_m:=20.0 \
    -p pos_z_m:=-30.0
```