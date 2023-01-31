# RumiCar ROS2 packages

## Get the source
```
mkdir -p ~/rcar/src
cd ~/rcar/src
git clone --recursive <url>/RumiCarRos.git
```

## Build
For building on RasPi Zero, you need to do [**cross-compilation**](docs/CrossCompilation.md).

After it was built the first time, next builds are done as simple as:
```
docker compose run --rm main

$ pibuild --packages-up-to rumicar
```

## Run
When packages are built (and uploaded), execute the following on the robot to start the driver:

```
. rcar_ros/setup.bash
ros2 run rumicar rumicar
```

## Joystick
Please, check [**here**](rumi_teleop/README.md).

## Video

This is an YouTube video for how this code works. -> [ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](https://youtu.be/bZCdvuuSebk)

[![ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](http://img.youtube.com/vi/bZCdvuuSebk/0.jpg)](https://youtu.be/bZCdvuuSebk "ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2")
