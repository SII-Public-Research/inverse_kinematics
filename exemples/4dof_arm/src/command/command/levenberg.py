"""
Date: 2023-03-27
Author: Romain
This script was developed for target positionning with different algorithms.
"""

import numpy as np

def non_linear_least_squares_functions(pos, joints):

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
        if np.linalg.norm(f(xt)) < np.linalg.norm(f(x)) :
            lam = 0.8*lam
            x = xt
        else:
            lam = 2.0*lam
    return x, {'Objective':obj, 'Residual':res}



if __name__ == '__main__':

    a0 = 0
    a1 = 3.75
    a2 = 3.25
    a3 = 6.05
    joints = [a0,a1,a2,a3]
    x = 0.87
    y = 0
    z = 10
    pos = [x,y,z] #x,y,z

    # Target location using L-M with LLS estimation as starting point and lambda = 0.1
    f, Df = non_linear_least_squares_functions(pos, joints)
    x_lm, history_lm = levenberg_marquardt(f, Df, [0,0,0], 0.1, 1e-4)
    #print(f(x_lls))
    print(np.degrees(x_lm) % 360)

    