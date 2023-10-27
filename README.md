# barista_robot_description
This work describes a robot model with plugins of differential drive and laser sensor

The xacro version allow anyone to set the geometry parameters of the robot

## Mandatory
+ For compiling
```
cd ~/ros2_ws/ ;colcon build --packages-select barista_robot_description; source install/setup.bash
```
+ For sourcing only
```
cd ~/ros2_ws/ ;source install/setup.bash
```

![imagen](https://github.com/Andy-Leo10/barista_robot_description/assets/60716487/8b9c0239-aa41-4a46-bfa7-24379349af67)

## Launch files
- [x] urdf version
```
ros2 launch barista_robot_description barista_urdf.launch.py
```
- [x] xacro version
```
ros2 launch barista_robot_description barista_xacro.launch.py
```

![imagen](https://github.com/Andy-Leo10/barista_robot_description/assets/60716487/a132e390-3ea3-4f2f-80f8-181c36cedc7c)

## Other for testing
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/barista1/cmd_vel
```
