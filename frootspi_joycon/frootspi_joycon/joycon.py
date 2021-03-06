#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2021 Roots
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


import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int16


class JoyCon(Node):

    def __init__(self):
        super().__init__('joycon')

        parameters = [
            ('button_chip_kick', 0),
            ('button_straight_kick', 1),
            ('button_dribble', 3),
            ('button_unlock_move', 4),
            ('button_unlock_kick', 5),
            ('axis_vel_sway', 0),
            ('axis_vel_surge', 1),
            ('axis_vel_angular', 2),
        ]
        self.declare_parameters('', parameters)

        self._BUTTON_CHIP_KICK = self.get_parameter('button_chip_kick').value
        self._BUTTON_STRAIGHT_KICK = self.get_parameter('button_straight_kick').value
        self._BUTTON_DRIBBLE = self.get_parameter('button_dribble').value
        self._BUTTON_UNLOCK_MOVE = self.get_parameter('button_unlock_move').value
        self._BUTTON_UNLOCK_KICK = self.get_parameter('button_unlock_kick').value
        self._AXIS_VEL_SWAY = self.get_parameter('axis_vel_sway').value
        self._AXIS_VEL_SURGE = self.get_parameter('axis_vel_surge').value
        self._AXIS_VEL_ANGULAR = self.get_parameter('axis_vel_angular').value

        self._sub_joy = self.create_subscription(
            Joy, 'joy', self._callback_joy, 1)

        self._pub_target_vel = self.create_publisher(Twist, 'target_velocity', 1)
        self._pub_dribble_pow = self.create_publisher(Float32, 'dribble_power', 1)
        self._pub_kick_pow = self.create_publisher(Float32, 'kick_power', 1)
        self._pub_kick_flag = self.create_publisher(Int16, 'kick_flag', 1)

        self._MAX_VEL_SURGE = 1.0
        self._MAX_VEL_SWAY = 1.0
        self._MAX_VEL_ANGULAR = math.pi

    def _callback_joy(self, msg):
        target_velocity = Twist()
        dribble_power = Float32()
        kick_power = Float32()
        kick_flag = Int16()

        # ????????????
        if msg.buttons[self._BUTTON_UNLOCK_MOVE]:
            target_velocity.linear.x = \
                msg.axes[self._AXIS_VEL_SURGE] * self._MAX_VEL_SURGE  # m/sec
            target_velocity.linear.y = \
                msg.axes[self._AXIS_VEL_SWAY] * self._MAX_VEL_SWAY  # m/sec
            target_velocity.angular.z = \
                msg.axes[self._AXIS_VEL_ANGULAR] * self._MAX_VEL_ANGULAR  # rad/sec
        self._pub_target_vel.publish(target_velocity)

        if msg.buttons[self._BUTTON_UNLOCK_KICK] and msg.buttons[self._BUTTON_DRIBBLE]:
            dribble_power.data = 0.5  # 0.0 ~ 1.0
        else:
            dribble_power.data = 0.0  # 0.0 ~ 1.0

        self._pub_dribble_pow.publish(dribble_power)

        # kick_command 6:???????????????, 5:?????????
        kick_command = \
            (int(msg.buttons[self._BUTTON_UNLOCK_KICK]) << 2)\
            + (int(msg.buttons[self._BUTTON_STRAIGHT_KICK]) << 1) \
            + int(msg.buttons[self._BUTTON_CHIP_KICK])

        # ???????????????
        if kick_command == 6:
            kick_power.data = 0.3  # m/s
            kick_flag.data = 1  # 0:??????, 1:???????????????, 2:?????????, 3:??????
        elif kick_command == 5:
            kick_power.data = 0.3  # m/s
            kick_flag.data = 2  # 0:??????, 1:???????????????, 2:?????????, 3:??????
        else:
            kick_power.data = 0.0  # m/s
            kick_flag.data = 0  # 0:??????, 1:???????????????, 2:?????????, 3:??????

        self._pub_kick_pow.publish(kick_power)
        self._pub_kick_flag.publish(kick_flag)


def main(args=None):
    rclpy.init(args=args)

    joycon = JoyCon()

    rclpy.spin(joycon)

    joycon.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
