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
~/battery_led_pins: (int[])
    gpio pins for battery indication
    
~/battery_led_voltages: (float[])
    [V] battery voltages from min to max
   
~/drive_gpios: (int[])
    gpio pins for driving
    PhEn => LR_SELECT, STEER_ON, DRIVE_ON, FORWARD_ON
    InIn => LEFT, RIGHT
    
~/drive_pwms: (int[])
    pwm indexes used for drive motor (either 0 for PhEn or 0,1 for InIn)
      
~/drive_pwm_frequency: (int)
    [Hz] PWM frequency for drive motor (lower - lower speeds, higher - smoother movement)

~/beeper_pwm: (int)
    PWM index used for beeper (-1 to disable)
```

