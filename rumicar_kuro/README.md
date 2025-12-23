# RumiCar Kuro

Experimental model with 2D lidar and other sensors.

Please, use it as reference for your own experiments.

## 注意
このパッケージは本家のラズパイRumiCarのピンアサインとは違います。

本家のラズパイRumiCarでこのROSのプログラムを使用する場合、RumiCarのFacebook groupでご相談ください。

RumiCar FacebookのURL：https://www.facebook.com/groups/rumicar

参考：https://github.com/RumiCar-group/RumiCar/tree/master/RasPi

### PWM
Edit `/boot/firmware/config.txt`:
```
# PWM for 18 and 13 pins
dtoverlay=pwm-2chan,pin=18,func=2,pin2=13,func2=4
```

## Run
```
ros2 launch rumicar_kuro kuro.launch
```
