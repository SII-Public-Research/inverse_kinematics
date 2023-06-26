#!/usr/bin/env python

# See related drawio file to get the different distances and robot initial position

import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from messages.msg import MotorsAngles

import numpy as np
import sympy as sp
import math

# Here we define all the constants we have on the arm
a1 = 95.0
a2 = 105.0
a3 = 100.0 + 40.0

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

        self.get_logger().info('received message', once=False)
        self.get_logger().info('Objectif :', once=False)
        self.get_logger().info(f'X = {x}', once=False) 
        self.get_logger().info(f'Y = {y}', once=False) 
        self.get_logger().info(f'Z = {z}', once=False)
        self.get_logger().info('', once=False)

        # we get theta_1 by doing eqy / eqx
        theta_1 = math.atan2(y, x)
        theta_1_degree = np.degrees(theta_1)

        # we get theta3 from X² + Y² + Z² (and using sin(a-b))
        S3 = (a1**2 - a2**2 - a3**2 + x**2 + y**2 + z**2 - 2 * a1 * z) / (2 * a2 * a3)
        # we then know that cos² + sin² = 1, so we can have two values for C3
        C3_pos = np.sqrt(1 - S3**2)
        C3_neg = -np.sqrt(1 - S3**2)
        # We get theta_3 from tan2
        theta_3 = math.atan2(S3, C3_pos)
        theta_3_bis = math.atan2(S3, C3_neg)
        theta_3_degree = np.degrees(theta_3)

        # Finally we ask the help of phyton to solve a two equations with C2, S2
        k1 = a2 + a3 * np.sin(theta_3)
        k2 = a3 * np.cos(theta_3)
        C2 = sp.symbols('C2')
        S2 = sp.symbols('S2')
        Z = z - a1
        X = (x - y) / (np.cos(theta_1) - np.sin(theta_1))
        eq1 = sp.Eq(k2 * C2 - k1 * S2, X)
        eq2 = sp.Eq(k1 * C2 + k2 * S2, Z)

        eq = [eq1, eq2]
        sol = sp.solve(eq, C2, S2)
        theta_2 = math.atan2(sol[S2], sol[C2])
        theta_2_degree = np.degrees(theta_2)

        # Then, we publish to the motor command
        motors_angles = MotorsAngles()
        motors_angles.theta_1 = theta_1_degree
        motors_angles.theta_2 = theta_2_degree
        motors_angles.theta_3 = theta_3_degree
        motors_angles.theta_4 = 0.0
        self.get_logger().info(f"theta_1 = {theta_1_degree}, theta_2 = {theta_2_degree}, theta_3 = {theta_3_degree} ", once=False)

        self.publisher.publish(motors_angles)
            


def main(args=None):

    rclpy.init(args=args)

    algo_three_dof = Algo()

    
    rclpy.spin(algo_three_dof)

    algo_three_dof.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()

