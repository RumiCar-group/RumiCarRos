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
For convenience we mount `rcar_ros` directory, which is automatically updated with rumicar ros packages:
* (build) → `docker:~/rcar/install/`
* (volume) → `~/rcar/install/`
* (sshfs) → `rcar.local:/home/pi/rcar_ros/`

The sshfs mount is done on the developer PC:
```
mkdir -p ~/rcar/install
sshfs -o reconnect,follow_symlinks,allow_other pi@rcar.local:/home/pi/rcar_ros ~/rcar/install
```

## Start Docker
```
cd ~/rcar/src/RumiCarRos
docker compose run --rm main

# --rm option removes the container on exit 
```

## Build All
Docker starts in `~/rcar` directory by default. The following command will build all the packages in that directory, which are `RumiCarRos` contents.

```
pibuild
```