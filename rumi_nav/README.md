# RumiCar Navigation

Experimental package for testing navigation with ros2.

Requires ydlidar and ros package for using it.

## Setup
Please, check the [Ubuntu Notes](../docs/UbuntuNotes.md).

## Build
```
colcon build --packages-up-to rumi_nav
```

## Mapping
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
# for amcl
ros2 run nav2_map_server map_saver_cli -f ~/.ros/map

# for slam_toolbox
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '.ros/map'}"
```

## Navigation
Localization:
```
ros2 launch rumi_nav localization.launch
```

Navigation:
```
ros2 launch rumi_nav nav.py
```
