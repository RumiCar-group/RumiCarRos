# RumiCar ROS2 packages

## Get the source
```
mkdir -p ~/ros/src
cd ~/ros/src
git clone --recursive <url>/RumiCarRos.git
```

## Build
For building on RasPi Zero, you need to do [**cross-compilation**](docs/CrossCompilation.md).

## Run
When packages are built (and uploaded), execute the following on the robot to start the driver:

```
. rc/setup.bash
ros2 run rumicar rumicar
```

This is an YouTube video for how this code works. -> [ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](https://youtu.be/bZCdvuuSebk)

[![ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](http://img.youtube.com/vi/bZCdvuuSebk/0.jpg)](https://youtu.be/bZCdvuuSebk "ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2")
