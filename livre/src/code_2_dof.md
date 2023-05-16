# Code
```python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 09:34:49 2023

@author: clement.pene, romain.saboret
"""

import numpy as np
import sympy as sp
import math

# Here we input the goal target (mm)
x = 0.0
y = 0.0
z = 190.0
print('Objectif :')
print(f'Xm = {x}') 
print(f'Ym = {y}') 


# Here we define all the constants we have on the arm
a1 = 85.0
a2 = 105.0

# We first check if the command is a possible input. If not, we change the z target
z = np.sqrt(a2**2 - x**2 - y**2) + a1
print(f'Zm = {z}')
print('')

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
trans_mat_1_2 = np.array([[sp.cos(Theta_2), -sp.sin(Theta_2), 0, A2 * sp.cos(Theta_2)],
                        [sp.sin(Theta_2), sp.cos(Theta_2), 0, A2 * sp.sin(Theta_2)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

trans_mat_0_2 = trans_mat_0_1 @ trans_mat_1_2
#print(f'trans_mat_0_2 = {trans_mat_0_2}')


# I separate equation for (x, y, z) mostly to be able to simplify them after
eq1 = sp.Eq(trans_mat_0_2[0, 3], x)
eq2 = sp.Eq(trans_mat_0_2[1, 3], y)
eq3 = sp.Eq(trans_mat_0_2[2, 3], z)

sp.simplify(trans_mat_0_2[0, 3])
sp.simplify(trans_mat_0_2[1, 3])
sp.simplify(trans_mat_0_2[2, 3])



# We can have S2 with Z eq
S2 = (z - a1) / a2
# Then, we can get 2 values for C2 according to eq cos² + sin² = 1
C2_pos = np.sqrt(1 - S2**2)
C2_neg = -np.sqrt(1 - S2**2)
# Finally, we got theta_2 from atan2
theta_2 = math.atan2(S2, C2_pos)
theta_2_bis = math.atan2(S2, C2_neg)
print(f'theta_2 = {np.mod(np.degrees(theta_2), 360)}')
print(f'theta_2_bis = {np.mod(np.degrees(theta_2_bis), 360)}')
# Note that we have two possible values for theta_2. 
# We noticed that the first one is all the time working fine on our case


# We can have S1 from Y eq
S1 = y / (a2 * np.cos(theta_2))
# We can have C1 with X eq
C1 = x / (a2 * np.cos(theta_2))
# Finally, we got theta_2 from atan2
theta_1 = math.atan2(S1, C1)
print(f'theta_1 = {np.mod(np.degrees(theta_1), 360)}')
```