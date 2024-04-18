## Install ROS

* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## Dependencies
Try [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html). After it's installed, run:
```
cd ~/rcar
rosdep install --from-paths src -y --ignore-src
```

## Build

```
. /opt/ros/humble/setup.bash
cd ~/rcar
colcon build --packages-up-to rumicar
```
