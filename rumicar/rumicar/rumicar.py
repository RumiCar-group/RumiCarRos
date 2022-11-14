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

import time
import pigpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

LEFT = 0
CENTER = 1
RIGHT = 2

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
AIN_FREQUENCY = 490
BIN_FREQUENCY = 960

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
pi.set_PWM_frequency(AIN1, AIN_FREQUENCY)
pi.set_PWM_frequency(AIN2, AIN_FREQUENCY)


class RumiCar(Node):

    def __init__(self):
        super().__init__('rumicar')
        self.last_twist = self.get_clock().now()

        self.subscription = self.create_subscription(TwistStamped, 'cmd_vel', self.on_twist, 1)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_twist(self, message):
        self.last_twist = self.get_clock().now()
        self.get_logger().info('Accel: %f, Steer: %f' % (message.twist.linear.x, message.twist.angular.z))

        if message.twist.linear.x < 0:
            pi.hardware_PWM(BIN1, BIN_FREQUENCY, 0)
            pi.hardware_PWM(BIN2, BIN_FREQUENCY, int(-message.twist.linear.x * SPEED_FACTOR))
        else:
            pi.hardware_PWM(BIN1, BIN_FREQUENCY, int(message.twist.linear.x * SPEED_FACTOR))
            pi.hardware_PWM(BIN2, BIN_FREQUENCY, 0)

        if message.twist.angular.z < -0.25:
            pi.set_PWM_dutycycle(AIN1, 255)
            pi.set_PWM_dutycycle(AIN2, 0)
        elif message.twist.angular.z > 0.25:
            pi.set_PWM_dutycycle(AIN1, 0)
            pi.set_PWM_dutycycle(AIN2, 255)
        else:
            pi.set_PWM_dutycycle(AIN1, 0)
            pi.set_PWM_dutycycle(AIN2, 0)

    def on_timer(self):
        now = self.get_clock().now()
        if now - self.last_twist > rclpy.time.Duration(seconds=0.25):
            pi.hardware_PWM(BIN1, BIN_FREQUENCY, 0)
            pi.hardware_PWM(BIN2, BIN_FREQUENCY, 0)


def main(args=None):
    rclpy.init(args=args)

    pi.set_mode(BIN1, pigpio.INPUT)
    pi.set_mode(BIN2, pigpio.INPUT)

    rumicar = RumiCar()
    rclpy.spin(rumicar)
    rclpy.shutdown()

    pi.stop()


if __name__ == '__main__':
    main()
