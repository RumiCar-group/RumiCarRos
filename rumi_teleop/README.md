# RumiCar Teleoperation

## Build
For, RasPi Zero 1 it's easier to install joystick nodes on the remote PC from debian packages. 

But you can challenge to build needed nodes on the robot directly:
* [joystick_drivers](https://github.com/ros-drivers/joystick_drivers)
* [teleop_tools](https://github.com/ros-teleop/teleop_tools)

## Launch
To use joystick, run the following on remote PC (or on robot):

```
ros2 launch rumi_teleop teleop.launch
```