## Install ROS

* https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

Needed packages:
```
sudo apt install ros-jazzy-ros-base
```

## Dependencies
Try [rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)
```
sudo rosdep init
rosdep update
cd ~/rcar
rosdep install -r --from-paths src -y --ignore-src
```

## Build

```
. /opt/ros/jazzy/setup.bash
cd ~/rcar
colcon build --packages-up-to rumicar
```
