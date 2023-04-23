# RumiCar driver

## Subscriptions

```
/cmd_vel [geometry_msgs::msg::Twist]
    velocity: 0.0 ~ 1.0 (max)
    steering: <0 = left, 0 = straight, >0 = right
    
/joy [sensor_msgs::msg::Joy]
    axes[1]: 0.0 ~ 1.0 (max), 70% cut
    axes[2]: <0 = left, 0 = straight, >0 = right  
```