# 注意
このプログラムは本家のラズパイRumiCarのピンアサインとは違います。

本家のラズパイRumiCarでこのROSのプログラムを使用する場合、RumiCarのFacebook groupでご相談ください。

RumiCar FacebookのURL：https://www.facebook.com/groups/rumicar

参考：https://github.com/RumiCar-group/RumiCar/tree/master/RasPi

# RumiCar ROS2

## Get the source
```
mkdir -p ~/rcar/src
cd ~/rcar/src
git clone https://github.com/RumiCar-group/RumiCarRos.git
```

## Robot Setup
Setup the robot micro SD card with [Ubuntu 24 Server](docs/UbuntuNotes.md)

* https://www.raspberrypi.com/software/

### Gpiod
The driver uses `gpiod` and `sysfs` to control the motors with minimum CPU consumption. 
If it was installed, `pigpio` should be turned off and uninstalled first.
```
sudo systemctl stop pigpiod
sudo systemctl disable pigpiod
sudo apt remove pigpiod
```

### PWM
Edit `/boot/firmware/config.txt`:
```
# PWM for 13 and 18 pins
dtoverlay=pwm-2chan,pin=18,func=2,pin2=13,func2=4
```

* https://github.com/dotnet/iot/blob/main/Documentation/raspi-pwm.md

## Build
* [RasPi Zero](docs/RasPiZero.md)
* [RasPi Zero 2](docs/RasPiZero2.md)

## Run
When packages are built (and uploaded), execute the following on the robot to start the driver:

```
. rcar/install/setup.bash
ros2 run rumi_driver rumi_driver

# or
ros2 launch rumicar original.launch
```

## Joystick
Please, check [**here**](rumi_teleop/README.md).

## Camera

* https://index.ros.org/r/v4l2_camera/

## Video

This is an YouTube video for how this code works. -> [ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](https://youtu.be/bZCdvuuSebk)

[![ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](http://img.youtube.com/vi/bZCdvuuSebk/0.jpg)](https://youtu.be/bZCdvuuSebk "ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2")
