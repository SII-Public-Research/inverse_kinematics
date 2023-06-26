#!/usr/bin/env python

# See related drawio file to get the different distances and robot initial position

import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from messages.msg import MotorsAngles

import numpy as np
import math

# Here we define all the constants we have on the arm
a1 = 95.0
a2 = 105.0 + 100.0 + 40.0

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
        

        # We can have S2 with Z eq
        S2 = (z - a1) / a2
        # Then, we can get 2 values for C2 according to eq cos² + sin² = 1
        C2_pos = np.sqrt(1 - S2**2)
        C2_neg = -np.sqrt(1 - S2**2)
        # Finally, we got theta_2 from atan2
        theta_2 = math.atan2(S2, C2_pos)
        theta_2_bis = math.atan2(S2, C2_neg)
        # Note that we have two possible values for theta_2.
        # We noticed that the first one is all the time working fine on our case

        # We can have S1 from Y eq
        S1 = y / (a2 * np.cos(theta_2))
        # We can have C1 with X eq
        C1 = x / (a2 * np.cos(theta_2))
        # Finally, we got theta_2 from atan2
        theta_1 = math.atan2(S1, C1)

        theta2_correct_degree = np.mod(np.degrees(theta_2), 360)
        theta1_correct_degree = np.mod(np.degrees(theta_1), 360)


        # I have a problem on theta1 that can be > 180, AND MY SERVO CANNOT
        if theta1_correct_degree > 180:
            theta1_correct_degree -= 180
            theta2_correct_degree = 180 - theta2_correct_degree

        # Then, we publish to the motor command
        motors_angles = MotorsAngles()
        motors_angles.theta_1 = theta1_correct_degree
        motors_angles.theta_2 = theta2_correct_degree
        motors_angles.theta_3 = 90.0
        motors_angles.theta_4 = 0.0
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

