#!/usr/bin/env python
# coding:utf-8
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
#from sys.float_info import epsilon
epsilon = sys.float_info.epsilon

fig = plt.figure()
ax1 = fig.add_subplot(221, projection='3d')
ax1.set_xlabel('X Label')
ax1.set_ylabel('Y Label')
ax1.set_title('3D point')

ax2 = fig.add_subplot(223)
ax2.axis([-5, 5, -5, 5])
ax2.set_xlabel('u Label')
ax2.set_ylabel('v Label')
ax2.set_title('Use Pinhole Camera Model')

ax3 = fig.add_subplot(224)
ax3.axis([-5, 5, -5, 5])
ax3.set_xlabel('u Label')
ax3.set_ylabel('v Label')
ax3.set_title('Use Stereographic projection Camera Model')

n = 100
"""
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
"""
##1,2,3,4,5,6,7
Xf = np.array([ -3, -2, -1, 0, 1, 2, 3])

Yf = np.array([ 1, 1, 1, 1, 1, 1, 1])

Zf = np.array([ 1, 1, 1, 1, 1, 1, 1])


X = np.array([])
Y = np.array([])
Z = np.array([])

X= np.append(X,Xf)
Y= np.append(Y,Yf)
Z= np.append(Z,Zf)

#= np.append(X,Xt)
#Y= np.append(Y,Yt)
#Z= np.append(Z,Zt)

#X= np.append(X,Xb)
#Y= np.append(Y,Yb)
#Z= np.append(Z,Zb)

ax1.scatter(Xf, Yf, Zf, c='r', marker='o')
#ax1.scatter(Xt, Yt, Zt, c='g', marker='o')
#ax1.scatter(Xb, Yb, Zb, c='b', marker='o')
ax1.scatter(0, 0, 0, c='b', marker='^', s=50)

C = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

for i in range(len(X)):
    x = X[i]
    y = Y[i]
    z = Z[i]
    p = np.matrix([[x], [y], [z]])
    U = C*p
    U = U/(U[2] + epsilon)
    u = U[0]
    v = U[1]

    ax2.scatter(u, v, c = 'b')
        

for i in range(len(X)):
    x = X[i]
    y = Y[i]
    z = Z[i]
    p = np.matrix([[x], [y], [z]])
    l = np.linalg.norm(p)
    theta = math.acos(z/l)
    phi = math.acos(x/(math.sqrt(x**2 + y**2) + epsilon))
    if y < 0:
        phi = -phi        
    r = 2*math.tan(theta/2)
    u = r*math.cos(phi)
    v = r*math.sin(phi)
    ax3.scatter(u, v, c = 'r')
    
    r_ = math.sqrt(u**2 + v**2)
    z_ = math.cos(2*math.atan(r/2))
    x_ = u * math.sqrt(1 - z_**2)/(r_ + epsilon)
    y_ = v * math.sqrt(1 - z_**2)/(r_ + epsilon)
    print '------------------------'
    print 'x:', x_/z_
    print 'y:', y_/z_
    print 'z:', 1
    

plt.show()
