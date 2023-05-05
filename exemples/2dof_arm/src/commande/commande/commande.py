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

        self.get_logger().info('received message', once=False)
        self.get_logger().info('Objectif :', once=False)
        self.get_logger().info(f'X = {x}', once=False) 
        self.get_logger().info(f'Y = {y}', once=False) 
        self.get_logger().info(f'Z = {z}', once=False)
        self.get_logger().info('', once=False)

        # We first check if the command is a possible input 
        if x**2 + y**2 + (z - a1)**2 != a2**2:
            self.get_logger().info('Input command is not valid', once=False)
            # If the command is not valid, we propose to keep x and y and adjust z to a possible value. 
            z = np.sqrt(a2**2 - x**2 - y**2) + a1
            self.get_logger().info(f'We corrected z value to: {z}', once=False)
        
        # we have two equations for theta_2, giving cos or sin. We want both to select the correct angle from trigo circle
        theta_2 = np.arcsin((z - a1) / a2)
        sol_theta_2_sin = [theta_2, math.pi - theta_2]
        sol_sin2_degree = [np.degrees(theta_2), np.degrees(math.pi - theta_2)]
        theta_2 = np.arccos(np.sqrt((x**2 + y**2) / a2**2))
        sol_theta_2_cos = [theta_2, -theta_2]
        sol_cos2_degree = [np.degrees(theta_2), np.degrees(-theta_2)]
        theta2_correct = sorted(product(np.mod(sol_theta_2_sin, 2*np.pi), np.mod(sol_theta_2_cos, 2*np.pi)), key=lambda t: abs(t[0]-t[1]))[0][0]
        theta2_correct_degree = sorted(product(np.mod(sol_sin2_degree, 360), np.mod(sol_cos2_degree, 360)), key=lambda t: abs(t[0]-t[1]))[0][0]


        # same here for theta_1
        theta_1 = np.arcsin(y / (a2 * np.cos(theta2_correct)))
        sol_theta_1_sin = [theta_1, math.pi - theta_1]
        sol_sin1_degree = [np.degrees(theta_1), np.degrees(math.pi - theta_1)]
        theta_1 = np.arccos(round(x/(a2*np.cos(theta2_correct)), 4))
        sol_theta_1_cos = [theta_1, -theta_1]
        sol_cos1_degree = [np.degrees(theta_1), np.degrees(-theta_1)]
        #theta1_correct = sorted(product(np.mod(sol_theta_1_sin, np.pi), np.mod(sol_theta_1_cos, np.pi)), key=lambda t: abs(t[0]-t[1]))[0][0]
        theta1_correct_degree = sorted(product(np.mod(sol_sin1_degree, 360), np.mod(sol_cos1_degree, 360)), key=lambda t: abs(t[0]-t[1]))[0][0]

        # I have a problem on theta1 that can be > 180, AND MY SERVO CANNOT
        if theta1_correct_degree > 180:
            theta1_correct_degree -= 180
            theta2_correct_degree = 180 - theta2_correct_degree

        # Then, we publish to the motor command
        motors_angles = MotorsAngles()
        motors_angles.theta_1 = theta1_correct_degree
        motors_angles.theta_2 = theta2_correct_degree
        self.get_logger().info(f"theta_1 = {theta1_correct_degree}, theta_2 = {theta2_correct_degree} ", once=False)

        self.publisher.publish(motors_angles)
            


def main(args=None):

    rclpy.init(args=args)

    algo_two_ddl = Algo()

    
    rclpy.spin(algo_two_ddl)

    algo_two_ddl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()

