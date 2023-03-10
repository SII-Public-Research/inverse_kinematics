#!/usr/bin/env python

import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from messages.msg import MotorsAngles

import numpy as np
import math

# Here we define all the constants we have on the arm
a1 = 100.0
a2 = 100.0
a3 = 100.0

class Algo(rclpy.node.Node):
    def __init__(self):
        super().__init__('algoTwoDdl')
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

        # We first check if the command is a possible input 
        if x**2 + y**2 + (z - a1)**2 != a2**2:
            print('Input command is not valid')
        else:
            theta_2 = np.arcsin((z - a1) / a2)
            sol_theta_2_sin = [round(theta_2, 2), round(math.pi - theta_2, 2)]
            sol_sin2_degree = [round(np.degrees(theta_2), 2), round(np.degrees(math.pi - theta_2), 2)]

            theta_2 = np.arccos(np.sqrt((x**2 + y**2) / a2**2))
            sol_theta_2_cos = [round(theta_2, 2), round(-theta_2, 2)]
            sol_cos2_degree = [round(np.degrees(theta_2), 2), round(np.degrees(-theta_2), 2)]

            theta2_correct = list(set(np.mod(sol_theta_2_sin, np.pi)).intersection(np.mod(sol_theta_2_cos, np.pi)))[0]
            theta2_correct_degree = np.degrees(theta2_correct)
            theta_1 = np.arcsin(y / (a2 * np.cos(theta2_correct)))
            sol_theta_1_sin = [round(theta_1, 2), round(math.pi - theta_1, 2)]
            sol_sin1_degree = [round(np.degrees(theta_1), 2), round(np.degrees(math.pi - theta_1), 2)]
            theta_1 = np.arccos(x/(a2*np.cos(theta2_correct)))
            sol_theta_1_cos = [round(theta_1, 2), round(-theta_1, 2)]
            sol_cos1_degree = [round(np.degrees(theta_1), 2), round(np.degrees(-theta_1), 2)]

            theta1_correct = list(set(np.mod(sol_theta_1_sin, np.pi)).intersection(np.mod(sol_theta_1_cos, np.pi)))[0]
            theta1_correct_degree = np.degrees(theta1_correct)

            motors_angles = MotorsAngles()
            motors_angles.theta_1 = theta1_correct_degree
            motors_angles.theta_2 = theta2_correct_degree

            self.publisher.publish(motors_angles)
            


def main(args=None):

    rclpy.init(args=args)

    algo_two_ddl = Algo()

    
    rclpy.spin(algo_two_ddl)

    algo_two_ddl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()

