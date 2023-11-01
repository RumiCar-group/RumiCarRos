# RumiCar ROS2 packages

## Get the source
```
mkdir -p ~/rcar/src
cd ~/rcar/src
git clone --recursive <url>/RumiCarRos.git
```

## Dependencies
### On development PC
Try [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html). After it's installed, run:
```
cd ~/rcar
rosdep install --from-paths src -y --ignore-src
```

### On RumiCar
Install needed runtime dependencies manually.

The driver uses `gpiod` and `sysfs` to control the motors with minimum CPU consumption. But in this case `pigpio` should be turned off and maybe uninstalled.
```
sudo systemctl stop pigpiod
sudo systemctl disable pigpiod
sudo apt remove pigpiod
```

PWM for 13 and 18 pins should be enabled in `/boot/config.txt`:
```
dtoverlay=pwm-2chan,pin=18,func=2,pin2=13,func2=4
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
. rcar/install/setup.bash
ros2 run rumi_driver rumi_driver

# or
ros2 launch rumicar main.launch
```

## Joystick
Please, check [**here**](rumi_teleop/README.md).

## Video

This is an YouTube video for how this code works. -> [ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](https://youtu.be/bZCdvuuSebk)

[![ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2](http://img.youtube.com/vi/bZCdvuuSebk/0.jpg)](https://youtu.be/bZCdvuuSebk "ROS2でRumiCarを遠隔操作/Remote control RumiCar by ROS2")

## Ubuntu Specific

Raspi Config is not preinstalled:
```
sudo apt-get install raspi-config
```

The path to `/boot/config.txt` is `/boot/firmware/config.txt`.

Udev rules need to be added manually: https://github.com/dotnet/iot/blob/main/Documentation/raspi-pwm.md

Fix the "System is booting up. Unprivileged users are not permitted to log in yet.": 
```
sudo nano /etc/pam.d/login
# comment out:  auth  requisite  pam_nologin.so
```

