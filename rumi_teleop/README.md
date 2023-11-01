# RumiCar Teleoperation

Proceed below if you connect joystick to remote PC.

## Build
```
cd rcar  # name of your workspace
rosdep install --from-paths src -y --ignore-src
ros2 colcon --packages-up-to rumi_teleop
```

## Launch
To use joystick connected to remote PC, run the following on it:

```
ros2 launch rumi_teleop teleop.launch
```
