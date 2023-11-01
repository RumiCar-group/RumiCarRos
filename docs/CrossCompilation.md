# Cross Compilation

Motivation: ~5 times faster.

## Check Connection to Robot
```
ssh pi@rcar.local
```

* here and below user name is set to `pi` and host name to `rcar.local`.

## Build ROS2 for RasPi Zero
Please, see the [**instruction**](https://github.com/nyacpp/raspi_cross_ros2).

As a result you should have `~/raspi` with crosscompiler and  `~/ros_humble/install` with ros built inside.

## Configure Automatic Upload
For convenience we mount `rcar/install` directory, which is automatically updated with rumicar ros packages:
* (build) → `docker:~/rcar/install/`
* (sshfs) → `rcar.local:/home/pi/rcar/install/`

## Start Docker
```
cd ~/rcar/src/RumiCarRos
docker compose run --rm main

# --rm option removes the container on exit 
```

## Build All
Docker starts in `~/rcar` ROS workspace. The following command builds packages there (the arguments are passed to underlying `colcon build`).

```
$ pibuild --packages-up-to rumicar
$ pibuild  # to build all
```