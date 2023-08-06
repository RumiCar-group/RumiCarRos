# RumiCar driver

Allows to control the robot and get its status.

## Subscriptions
```
/cmd_vel [geometry_msgs::msg::Twist]
    velocity: 0.0 ~ 1.0 (max)
    steering: <0 = left, 0 = straight, >0 = right
    
/joy [sensor_msgs::msg::Joy]
    axes[1]: 0.0 ~ 1.0 (max), 70% cut
    axes[2]: <0 = left, 0 = straight, >0 = right  
```

## Publications
```
/battery [sensor_msgs::msg::BatteryState]
    battery voltage and percentage
```

## Parameters
```
~/steering_pins: (int[], default: [ 17, 27 ])
    gpio pins used for steering left and right
    
~/battery_led_pins: (int[], default: [ 5, 6, 16, 20, 21 ])
    gpio pins for battery indication
    
~/battery_led_voltages: (float[], default: [ 2.8, 3.2, 3.4, 3.6 ])
    [V] battery voltages for each pin after the first
     
~/drive_pwm_frequency: (int, default: 36)
    [Hz] PWM frequency for drive motor (lower - lower speeds, higher - smoother movement)
```
