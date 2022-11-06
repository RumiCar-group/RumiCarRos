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
