# Code

Fonctions de l'implémentation de **Levenberg-Marquart** et de sa fonction assosiée au problème [4 DOF](./4_dof.md).

En dessous se trouve le code principal.

```python
"""
Date: 2023-03-27
Author: Romain
This script was developed for target positionning with different algorithms.
"""

import numpy as np

def non_linear_least_squares_functions(pos, joints):
    print(f'{joints}')

    # f(x) returns the residuals
    f = lambda x: np.array([joints[1]*np.cos(x[0]) + joints[2]*np.cos(x[0]+x[1]) + joints[3]*np.cos(x[0]+x[1]+x[2]) - pos[0],
                joints[0] + joints[1]*np.sin(x[0]) + joints[2]*np.sin(x[0]+x[1]) + joints[3]*np.sin(x[0]+x[1]+x[2]) - pos[2]])
    # Df(x) is the N by 2 derivative matrix
    Df = lambda x: np.array([[-joints[1]*np.sin(x[0]) - joints[2]*np.sin(x[0]+x[1]) - joints[3]*np.sin(x[0]+x[1]+x[2]), -joints[2]*np.sin(x[0]+x[1]) - joints[3]*np.sin(x[0]+x[1]+x[2]), -joints[3]*np.sin(x[0]+x[1]+x[2])],
                             [joints[1]*np.cos(x[0]) + joints[2]*np.cos(x[0]+x[1]) + joints[3]*np.cos(x[0]+x[1]+x[2]), joints[2]*np.cos(x[0]+x[1]) + joints[3]*np.cos(x[0]+x[1]+x[2]), joints[3]*np.cos(x[0]+x[1]+x[2])]])
    return f, Df

def levenberg_marquardt(f, Df, x0, lambda0, tol = 1e-2, kmax = 100):
    n = len(x0)
    x = x0
    lam = lambda0
    obj = np.zeros((0,1))
    res = np.zeros((0,1))
    for k in range(kmax):
        obj = np.vstack([obj, np.linalg.norm(f(x))**2])
        res = np.vstack([res, np.linalg.norm(2*Df(x).T @ f(x))])
        if np.linalg.norm(2*Df(x).T @ f(x)) < tol:
            break
        xt = x - np.linalg.inv((Df(x).T @ Df(x)) + lam*np.eye(n)) @ (Df(x).T @ f(x))
        
        # handle 180° limit with 20° marge error (~10%*pi)
        xt[0] = np.clip(xt[0],  0.1 * np.pi, 0.9 * np.pi) #   0° - 180°
        xt[1] = np.clip(xt[1], -0.4 * np.pi, 0.4 * np.pi) # -90° - 90°
        xt[2] = np.clip(xt[2], -0.4 * np.pi, 0.4 * np.pi) # -90° - 90°
                

        if np.linalg.norm(f(xt)) < np.linalg.norm(f(x)) :
            lam = 0.8*lam
            x = xt
        else:
            lam = 2.0*lam
    return x, {'Objective':obj, 'Residual':res}
```

Et ici le code principal faisant appel au code précédent.
Ils vous faut mettre votre position x,y,z voulue dans les variables au début du programme.


```python
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
x_lm, history_lm = lv.levenberg_marquardt(f, Df, [0,0,0], 0.1, 1e-4)

thetas = np.concatenate([np.array([np.arctan2(y,x)*180/np.pi % 360]), np.degrees(x_lm)])
print(f"{thetas}")        
print(f"{thetas % 360}")

# I have a problem on theta1 that can be > 180, AND MY SERVO CANNOT
if thetas[0] > 180:
    thetas[0] -= 180
    thetas[1] = 180 - thetas[1]
    thetas[2] = 180 - thetas[2]
    thetas[3] = 180 - thetas[3]


print(f"theta_1 = {thetas[0]}, theta_2 = {thetas[1]}, theta_3 = {thetas[2]}, theta_4 = {thetas[3]} ")
```