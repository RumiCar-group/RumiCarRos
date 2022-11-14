# Cross Compilation

Motivation: ~5 times faster.

## Check Connection to Robot
```
ssh pi@rcar.local
```

* here and below user name is set to `pi` and host name to `rcar.local`.

## Build ROS2 for RasPi Zero
Please, see the [**instruction**](../raspi_cross_ros2/README.md).

## Configure Automatic Upload
For convenience we mount `~/rc` directory, which mediates the automatic upload of the rumicar ros packages installation:
* `docker:~/ros/install/`
* (volume) → `~/rc/`
* (sshfs) → `rcar.local:/home/pi/rc/`

On the developer PC:
```
mkdir ~/rc
sshfs -o reconnect,follow_symlinks,allow_other pi@rcar.local:/home/pi/rc ~/rc
```

## Start Docker
```
cd ~/ros/src/RumiCarRos
docker compose run --rm main

# --rm option removes the container on exit 
```

## Build All
Docker starts in `~/ros` directory by default. The following command will build all the packages in that directory, which are `RumiCarRos` contents.

```
pibuild
```