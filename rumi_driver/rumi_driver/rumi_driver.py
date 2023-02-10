# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pigpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# DRV8835 pins
AIN1 = 17
AIN2 = 27
BIN1 = 18
BIN2 = 13

# VL53L0X pins
SHDN0 = 23
SHDN1 = 24
SHDN2 = 25

# PWM freq
BIN_FREQUENCY = 36  # lower - lower speeds, higher - smooth movement

# factors
SPEED_FACTOR = 1000000

pi = pigpio.pi()
pi.set_mode(AIN1, pigpio.OUTPUT)
pi.set_mode(AIN2, pigpio.OUTPUT)
pi.set_mode(BIN1, pigpio.OUTPUT)
pi.set_mode(BIN2, pigpio.OUTPUT)
pi.set_mode(SHDN0, pigpio.OUTPUT)
pi.set_mode(SHDN1, pigpio.OUTPUT)
pi.set_mode(SHDN2, pigpio.OUTPUT)


class RumiDriver(Node):

    def __init__(self):
        super().__init__('rumi_driver')
        self.last_time = self.get_clock().now()
        self.last_speed = 0

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.on_twist, 1)
        self.subscription = self.create_subscription(Joy, 'joy', self.on_joy, 1)

    def on_twist(self, message):
        self.last_time = self.get_clock().now()
        self.last_speed = message.linear.x
        self.get_logger().info('Accel: %f, Steer: %f' % (message.linear.x, message.angular.z))

        if message.linear.x < -0.15:
            pi.hardware_PWM(BIN1, BIN_FREQUENCY, 0)
            pi.hardware_PWM(BIN2, BIN_FREQUENCY, int(-message.linear.x * SPEED_FACTOR))
        elif message.linear.x > 0.15:
            pi.hardware_PWM(BIN1, BIN_FREQUENCY, int(message.linear.x * SPEED_FACTOR))
            pi.hardware_PWM(BIN2, BIN_FREQUENCY, 0)
        else:
            pi.hardware_PWM(BIN1, BIN_FREQUENCY, 0)
            pi.hardware_PWM(BIN2, BIN_FREQUENCY, 0)

        if message.angular.z < -0.2:
            pi.set_PWM_dutycycle(AIN1, 255)
            pi.set_PWM_dutycycle(AIN2, 0)
        elif message.angular.z > 0.2:
            pi.set_PWM_dutycycle(AIN1, 0)
            pi.set_PWM_dutycycle(AIN2, 255)
        else:
            pi.set_PWM_dutycycle(AIN1, 0)
            pi.set_PWM_dutycycle(AIN2, 0)

    def on_joy(self, message):
        twist = Twist()
        twist.linear.x = message.axes[1] * 0.3  # max 30%
        twist.angular.z = message.axes[2]
        self.on_twist(twist)


def main(args=None):
    rclpy.init(args=args)

    rumicar = RumiDriver()
    try:
        rclpy.spin(rumicar)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

    pi.stop()


if __name__ == '__main__':
    main()
