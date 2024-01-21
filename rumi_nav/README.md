# RumiCar Navigation

Experimental package for testing navigation with ros2.

Requires ydlidar and ros package for using it.

## Build
```
colcon build --packages-up-to rumi_nav
```

## Run
Source:
```
. ~/rcar/install/setup.bash
```

Mapping:
```
ros2 launch rumi_nav mapping.launch
```

Saving map:
```
ros2 run nav2_map_server map_saver_cli -f ~/.ros/map
```

Navigation:
```
ros2 launch rumi_nav nav.py
```
