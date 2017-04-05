#!/usr/bin/env python
# coding:utf-8

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math


class Optimizer:
    def __init__(self, x, y, arg):
        print ' Must define error and Jacobian'
        self.x = x
        self.y = y
        self.arg = arg

    def e(self):
        """ Define your error function"""
        return np.matrix([self.y - self.arg[0,0]*self.x**3 - self.arg[1,0]*self.x**2 - self.arg[2,0]*self.x**1 - self.arg[3,0]])

    def jacobian_i(self, x):
        """ Define your Jacobian function"""
        return np.matrix([x**3, x**2, x, 1])

    def solve(self):
        last_error = 10000000
        for j in range(10):
            e = self.e()
            error = np.sum(np.abs(e))
            if abs(last_error -  error) < 0.00001:
                break
            last_error =  error
            H = np.matrix(np.zeros((self.arg.size, self.arg.size)))
            B = np.matrix(np.zeros((self.arg.size, 1)))
            print 'error:', error
            for i in range(x.size):
                jacobian_i = self.jacobian_i(self.x[i]) 
                H = H + jacobian_i.T * jacobian_i
                B = B + jacobian_i.T * np.matrix([e[0,i]])
            darg = np.linalg.solve(H, B)
            self.arg = self.arg + darg
        return self.arg

x = np.arange(0,6,0.1)
arg = np.array([[1],[1],[1], [1]])
y = 1*x**3 - 2*x**2 - 4*x + 5
y_noise = y + np.random.rand(x.size) * 10 - 5
opt = Optimizer(x, y_noise, arg)
arg = opt.solve()
#y_predict = arg[0,0]*x**2 + arg[1,0]*x + arg[2,0]
y_predict = arg[0,0]*x**3 + arg[1,0]*x**2 + arg[2,0]*x + arg[3,0]
plt.plot(x, y_predict)
plt.plot(x, y_noise)
print arg
plt.show()

