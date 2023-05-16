#!/usr/bin/env python

# See related drawio file to get the different distances and robot initial position

import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from messages.msg import MotorsAngles
from . import levenberg as lv

import numpy as np
import math
from itertools import product

# Here we define all the constants we have on the arm
a1 = 85.0
a2 = 105.0
a3 = 100.0
a4 = 30.0
a5 = 50.0

class Algo(rclpy.node.Node):
    def __init__(self):
        super().__init__('algo_four_dof')
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

        pos = [x, y, z] # vecteur de la position voulue
        joints = [a1, a2, a3, a4] # vecteur avec la taille des différentes branches

        # Target location using L-M with LLS estimation as starting point and lambda = 0.1
        f, Df = lv.non_linear_least_squares_functions(pos, joints)
        x_lm, history_lm = lv.levenberg_marquardt_2(f, Df, [0,0,0], 0.1, 1e-4)

        thetas = np.concatenate([np.array([np.arctan2(y,x)*180/np.pi]), np.degrees(x_lm)])
        self.get_logger().info(f"{thetas}")        
        self.get_logger().info(f"{thetas % 360}")

        # I have a problem on theta1 that can be > 180, AND MY SERVO CANNOT
        if thetas[0] > 180:
            thetas[0] -= 180
            thetas[1] = 180 - thetas[1]
            thetas[2] = 180 - thetas[2]
            thetas[3] = 180 - thetas[3]


        # Then, we publish to the motor command
        motors_angles = MotorsAngles()
        motors_angles.theta_1 = thetas[0]
        motors_angles.theta_2 = thetas[1]
        motors_angles.theta_3 = thetas[2]
        motors_angles.theta_4 = thetas[3]
        self.get_logger().info(f"theta_1 = {thetas[0]}, theta_2 = {thetas[1]}, theta_3 = {thetas[2]}, theta_4 = {thetas[3]} ", once=False)

        self.publisher.publish(motors_angles)



def main(args=None):

    rclpy.init(args=args)

    algo_4dof = Algo()


    rclpy.spin(algo_4dof)

    algo_4dof.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()

