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
        return np.matrix([-x**3, -x**2, -x, -1])

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
            darg = np.linalg.solve(H, -B)
            self.arg = self.arg + darg
        return self.arg

x = np.arange(-3,3,0.1)
realVal = np.array([[3.],[-2.],[-4.], [5.]])
predVal = np.array([[1.],[1.],[1.], [1.]])
y = realVal[0,0]*x**3 + realVal[1,0]*x**2 + realVal[2,0]*x + realVal[3,0]
y_noise = y + np.random.rand(x.size) * 10 - 5
opt = Optimizer(x, y_noise, realVal)
predVal = opt.solve()
y_predict = predVal[0,0]*x**3 + predVal[1,0]*x**2 + predVal[2,0]*x + predVal[3,0]
predict, =       plt.plot(x, y_predict)
measurement, =   plt.plot(x, y_noise)
real, =          plt.plot(x, y)
plt.legend((predict, measurement, real),
   ('predict','measurement', 'real'),
   scatterpoints=1,
   loc='lower left',
   ncol=3,
   fontsize=11)
print 'real value:'
print realVal
print 'predicted value:'
print predVal
print 'error:'
print np.linalg.norm(realVal - predVal)
plt.show()

