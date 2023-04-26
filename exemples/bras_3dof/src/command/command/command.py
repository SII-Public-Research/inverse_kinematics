#!/usr/bin/env python

# See related drawio file to get the different distances and robot initial position

import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from messages.msg import MotorsAngles

import numpy as np
import math
from itertools import product

# Here we define all the constants we have on the arm
a1 = 100.0
a2 = 100.0

class Algo(rclpy.node.Node):
    def __init__(self):
        super().__init__('algoThreeDof')
        # create a publisher to put the values of motos angles
        self.publisher = self.create_publisher(
            MotorsAngles, 
            'motors_angles', 
            10) 
        # create a subscription to get the Point target (x, y, z)
        self.subscription = self.create_subscription(
            Point,
            'cmd_arm',
            self._cmd_received_callback,
            10)
        self.subscription # prevent unused variable warning

    def _cmd_received_callback(self, message):
        """Handle new command message."""

        x = message.x
        y = message.y
        z = message.z

        print('received message')
        print('Objectif :')
        print(f'X = {x}') 
        print(f'Y = {y}') 
        print(f'Z = {z}')
        print('')

        



        self.publisher.publish(motors_angles)
            


def main(args=None):

    rclpy.init(args=args)

    algo_three_dof = Algo()

    
    rclpy.spin(algo_three_dof)

    algo_three_dof.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()

