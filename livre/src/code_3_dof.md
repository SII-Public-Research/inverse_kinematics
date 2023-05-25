# Code
```python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 09:34:49 2023

@author: clement.pene
"""

import numpy as np
import sympy as sp
import math

# Here we input the goal target (mm)
x = 00.0
y = 00.0
z = 300.0

print('Objectif :')
print(f'Xm = {x}') 
print(f'Ym = {y}') 
print(f'Zm = {z}')
print('')

# Here we define all the constants we have on the arm
a1 = 85.0
a2 = 105.0
a3 = 100.0

# we want to use the transition matrix to get equations 
Theta_1 = sp.symbols('Theta_1')
Theta_2 = sp.symbols('Theta_2')
Theta_3 = sp.symbols('Theta_3')
A1 = sp.symbols('A1')
A2 = sp.symbols('A2')
A3 = sp.symbols('A3')
X = sp.symbols('X')
Y = sp.symbols('Y')
Z = sp.symbols('Z')


trans_mat_0_1 = np.array([[sp.cos(Theta_1), 0, sp.sin(Theta_1), 0],
                        [sp.sin(Theta_1), 0, -sp.cos(Theta_1), 0],
                        [0, 1, 0, A1],
                        [0, 0, 0, 1]])
trans_mat_1_2 = np.array([[sp.cos(Theta_2), -sp.sin(Theta_2), 0, -A2 * sp.sin(Theta_2)],
                        [sp.sin(Theta_2), sp.cos(Theta_2), 0, A2 * sp.cos(Theta_2)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
trans_mat_2_3 = np.array([[sp.cos(Theta_3), -sp.sin(Theta_3), 0, A3 * sp.cos(Theta_3)],
                        [sp.sin(Theta_3), sp.cos(Theta_3), 0, A3 * sp.sin(Theta_3)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

trans_mat_0_3 = trans_mat_0_1 @ trans_mat_1_2 @ trans_mat_2_3
#print(f'trans_mat_0_3 = {trans_mat_0_3}')


# I separate equation for (x, y, z) mostly to be able to simplify them after
eq1 = sp.Eq(trans_mat_0_3[0, 3], x)
eq2 = sp.Eq(trans_mat_0_3[1, 3], y)
eq3 = sp.Eq(trans_mat_0_3[2, 3], z)

sp.simplify(trans_mat_0_3[0, 3])
sp.simplify(trans_mat_0_3[1, 3])
sp.simplify(trans_mat_0_3[2, 3])


# we get theta_1 by doing eqy / eqx
theta_1 = math.atan2(y, x)
theta_1_degree = np.degrees(theta_1)
print(f'theta_1 = {theta_1}')
print(f'theta_1_degree = {theta_1_degree}')

# we get theta3 from X² + Y² + Z² (and using sin(a-b))
S3 = (a1**2 - a2**2 - a3**2 + x**2 + y**2 + z**2 - 2 * a1 * z) / (2 * a2 * a3)
# we then know that cos² + sin² = 1, so we can have two values for C3
C3_pos = np.sqrt(1 - S3**2)
print(f'C3_pos = {C3_pos}')
C3_neg = -np.sqrt(1 - S3**2)
print(f'C3_neg = {C3_neg}')
# We get theta_3 from tan2
theta_3 = math.atan2(S3, C3_pos)
theta_3_bis = math.atan2(S3, C3_neg)#print(f'theta_3 = {np.degrees(theta_3)}')
print(f'theta_3 = {np.degrees(theta_3)}')
print(f'theta_3_bis = {np.degrees(theta_3_bis)}')
# Note that both values are possibles positions for the arm

# To get theta_2, we first define some constantes to get simple equations
k1 = a2 + a3 * np.sin(theta_3)
k2 = a3 * np.cos(theta_3)
Z = z - a1
X = (x - y) / (np.cos(theta_1) - np.sin(theta_1))

C2 = sp.symbols('C2')
S2 = sp.symbols('S2')

# We use X and Z equations to get a system with 2 equations and 2 unknown values
# We can then ask the help of Python to get the result (thank you sympy)
eq1 = sp.Eq(k2 * C2 - k1 * S2, X)
eq2 = sp.Eq(k1 * C2 + k2 * S2, Z)
eq = [eq1, eq2]

sol = sp.solve(eq, C2, S2)
print(f'sol = {sol}')
theta_2 = math.atan2(sol[S2], sol[C2])
print(f'theta_2 = {np.degrees(theta_2)}')

print('')
print(f'theta_1 = {np.degrees(theta_1)}')
print(f'theta_2 = {np.degrees(theta_2)}')
print(f'theta_3 = {np.degrees(theta_3)}')
```