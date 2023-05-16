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
x = 30.0
y = 00.0
z = 270.0

print('Objectif :')
print(f'Xm = {x}') 
print(f'Ym = {y}') 
print(f'Zm = {z}')
print('')

# Here we define all the constants we have on the arm
a1 = 85.0
a2 = 105.0
a3 = 100.0
a4 = 30.0
a5 = 50.0


# we want to use the transition matrix to get equations 
Theta_1 = sp.symbols('Theta_1')
Theta_2 = sp.symbols('Theta_2')
Theta_3 = sp.symbols('Theta_3')
Theta_4 = sp.symbols('Theta_4')
A1 = sp.symbols('A1')
A2 = sp.symbols('A2')
A3 = sp.symbols('A3')
A4 = sp.symbols('A4')
A5 = sp.symbols('A5')
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
trans_mat_3_4 = np.array([[sp.cos(Theta_4), -sp.sin(Theta_4), 0, A4 * sp.cos(Theta_4) - A5 * sp.sin(Theta_4)],
                        [sp.sin(Theta_4), sp.cos(Theta_4), 0, A4 * sp.sin(Theta_4) + A5 * sp.cos(Theta_4)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

trans_mat_0_4 = trans_mat_0_1 @ trans_mat_1_2 @ trans_mat_2_3 @ trans_mat_3_4
# print(f'trans_mat_0_4 = {trans_mat_0_4}')


# I separate equation for (x, y, z) mostly to be able to simplify them after
eq1 = sp.Eq(trans_mat_0_4[0, 3], x)
eq2 = sp.Eq(trans_mat_0_4[1, 3], y)
eq3 = sp.Eq(trans_mat_0_4[2, 3], z)

sp.simplify(trans_mat_0_4[0, 3])
sp.simplify(trans_mat_0_4[1, 3])
sp.simplify(trans_mat_0_4[2, 3])

# one equation is very important now. We want that y4 = z0 (see schematics), so the point M is all the time facing up
sp.simplify(trans_mat_0_4[2, 1])



# we get theta_1 by doing eqy / eqx
theta_1 = math.atan2(y, x)
theta_1_degree = np.degrees(theta_1)
print(f'theta_1 = {theta_1}')
print(f'theta_1_degree = {theta_1_degree}')

# We have to make a correction on x and y because of a4 distance that we didn't include during 3 dof study
x = x - a4 * np.cos(theta_1)
y = y - a4 * np.sin(theta_1)
# We also make a correction on z, but through a1 (adding a5)
a1 += a5


# we get theta3 from X² + Y² + Z² (and using sin(a-b))
S3 = (a1**2 - a2**2 - a3**2 + x**2 + y**2 + z**2 - 2 * a1 * z) / (2 * a2 * a3)
# we then know that cos² + sin² = 1, so we can have two values for C3
C3_pos = np.sqrt(1 - S3**2)
print(f'C3_pos = {C3_pos}')
C3_neg = -np.sqrt(1 - S3**2)
print(f'C3_neg = {C3_neg}')
# We get theta_3 from tan2
theta_3 = math.atan2(S3, C3_pos)
theta_3_bis = math.atan2(S3, C3_neg)
print(f'theta_3 = {np.degrees(theta_3)}')
print(f'theta_3_bis = {np.degrees(theta_3_bis)}')
# Note that both values are possibles positions for the arm

# To get theta_1, we first define some constantes to get simple equations
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


# we solve theta_4 by using trans_mat_0_4[2, 1]
# cos(Theta_2 + Theta_3 + Theta_4) = 1
theta_4 = -theta_2 - theta_3 
print(f'theta_4 = {np.degrees(theta_4)}')
```