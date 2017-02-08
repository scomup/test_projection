#!/usr/bin/env python
# coding:utf-8

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math


fig = plt.figure()
ax1 = fig.add_subplot(221, projection='3d')
ax1.set_xlabel('X Label')
ax1.set_ylabel('Y Label')
ax1.set_title('3D point')

ax2 = fig.add_subplot(223)
ax2.axis([0, 10, 0, 10])
ax2.set_xlabel('u Label')
ax2.set_ylabel('v Label')
ax2.set_title('Use Pinhole Camera Model')

ax3 = fig.add_subplot(224)
ax3.axis([0, 10, 0, 10])
ax3.set_xlabel('u Label')
ax3.set_ylabel('v Label')
ax3.set_title('Use Stereographic projection Camera Model')

n = 100

Xf = np.array([ -3, -3, -3, -3, -3, -3, -3,
                -2, -2, -2, -2, -2, -2, -2,
                -1, -1, -1, -1, -1, -1, -1,
                0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1, 1,
                2, 2, 2, 2, 2, 2, 2,
                3, 3, 3, 3, 3, 3, 3])

Yf = np.array([ -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3])


Zf = np.array([ 3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3])

Xt = np.array([ -3, -3, -3, -3, -3, -3, -3,
                -2, -2, -2, -2, -2, -2, -2,
                -1, -1, -1, -1, -1, -1, -1,
                0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1, 1,
                2, 2, 2, 2, 2, 2, 2,
                3, 3, 3, 3, 3, 3, 3])


Yt = np.array([ -3, -3, -3, -3, -3, -3, -3,
                -3, -3, -3, -3, -3, -3, -3,
                -3, -3, -3, -3, -3, -3, -3,
                -3, -3, -3, -3, -3, -3, -3,
                -3, -3, -3, -3, -3, -3, -3,
                -3, -3, -3, -3, -3, -3, -3,
                -3, -3, -3, -3, -3, -3, -3])

Zt = np.array([ -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3])


Xb = np.array([ -3, -3, -3, -3, -3, -3, -3,
                -2, -2, -2, -2, -2, -2, -2,
                -1, -1, -1, -1, -1, -1, -1,
                0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1, 1,
                2, 2, 2, 2, 2, 2, 2,
                3, 3, 3, 3, 3, 3, 3])


Yb = np.array([ 3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3])

Zb = np.array([ -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3,
                -3, -2, -1, 0, 1, 2, 3])


X = np.array([])
Y = np.array([])
Z = np.array([])

X= np.append(X,Xf)
Y= np.append(Y,Yf)
Z= np.append(Z,Zf)

X= np.append(X,Xt)
Y= np.append(Y,Yt)
Z= np.append(Z,Zt)

X= np.append(X,Xb)
Y= np.append(Y,Yb)
Z= np.append(Z,Zb)

ax1.scatter(Xf, Yf, Zf, c='r', marker='o')
ax1.scatter(Xt, Yt, Zt, c='g', marker='o')
ax1.scatter(Xb, Yb, Zb, c='b', marker='o')
ax1.scatter(0, 0, 0, c='b', marker='^', s=50)

C = np.matrix([[2, 0, 5], [0, 2, 5], [0, 0, 1]])

for i in range(len(X)):
    p = np.matrix([[X[i]], [Y[i]], [Z[i]]])
    U = C*p
    U = U/U[2]
    u = U[0]
    v = U[1]
    if Z[i] == 3:
        ax2.scatter(u, v, c = 'r')
    elif Y[i] == -3 and Z[i] > 0:
        ax2.scatter(u, v, c = 'g')
        pass
    elif Y[i] == 3 and Z[i] > 0:
        pass
        ax2.scatter(u, v, c = 'b')
        

for i in range(len(X)):
    p = np.matrix([[X[i]], [Y[i]], [Z[i]]])
    l = np.linalg.norm(p)
    theta = math.acos(p[2, 0]/l)
    phi = math.acos(p[0, 0]/math.sqrt(p[0, 0]**2 + p[1, 0]**2))
    if p[1, 0] < 0:
        phi = -phi        
    r = 2*math.tan(theta/2)
    u = r*math.cos(phi) + 5
    v = r*math.sin(phi) + 5
    if Z[i] == 3:
        ax3.scatter(u, v, c = 'r')
    elif Y[i] == -3:
        ax3.scatter(u, v, c = 'g')
    elif Y[i] == 3:
        ax3.scatter(u, v, c = 'b')
plt.show()
