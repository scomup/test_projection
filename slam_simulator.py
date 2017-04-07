#!/usr/bin/env python
# coding:utf-8
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from math import *
#from sys.float_info import epsilon
epsilon = sys.float_info.epsilon



class Optimizer:
    def __init__(self):
        print ' Must define error and Jacobian'
        
    def set(self, Y, mapPoint, arg, fx, fy, cx, cy, yaw, twc):
        self.mapPoint = mapPoint
        #self.X = X
        self.Y = Y
        self.arg = arg
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.yaw = yaw
        self.twc = twc
        self.arg = arg


        


    def e(self,Y_i,mapPoint):
        """ Define your error function"""
        u = Y_i[0]
        v = Y_i[1]
        x = mapPoint[0]
        y = mapPoint[1]
        z = mapPoint[2]
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy
        yaw = self.yaw
        xo = self.twc[0,0]
        yo = self.twc[1,0]
        dyaw = self.arg[2,0]
        dx = self.arg[0,0]
        dy = self.arg[1,0]
        uu = (fx/z)*(cos(yaw + dyaw)*(x - xo - dx) + sin(yaw + dyaw)*(y - yo - dy)) + cx
        vv = (fy/z)*(-sin(yaw + dyaw)*(x - xo - dx) + cos(yaw + dyaw)*(y - yo - dy)) + cy
        return np.matrix([[u - (fx/z)*(cos(yaw + dyaw)*(x - xo - dx) + sin(yaw + dyaw)*(y - yo - dy)) - cx],
                          [v - (fy/z)*(-sin(yaw + dyaw)*(x - xo - dx) + cos(yaw + dyaw)*(y - yo - dy)) - cy]])

    def jacobian_i(self, mapPoint):
        """ Define your Jacobian function"""
        # u - (fx/z)*cos(yaw + dyaw)*(x - xo - dx) -(fx/z) sin(yaw + dyaw)*(y - yo - dy)) - cx
        # v + (fy/z)*sin(yaw + dyaw)*(x - xo - dx) -(fy/z) cos(yaw + dyaw)*(y - yo - dy)) - cy
        x = mapPoint[0]
        y = mapPoint[1]
        z = mapPoint[2]
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy
        yaw = self.yaw
        xo = self.twc[0,0]
        yo = self.twc[1,0]
        dyaw = self.arg[2,0]
        dx = self.arg[0,0]
        dy = self.arg[1,0]
        J = np.matrix(np.zeros((2, 3)))
        J[0,0] = (fx/z)*cos(yaw + dyaw)
        J[0,1] = (fx/z)*sin(yaw + dyaw)
        J[0,2] = (fx/z)*sin(yaw + dyaw)*(x - xo - dx) - (fx/z)*cos(yaw + dyaw)*(y - yo - dy)
        J[1,0] = -(fy/z)*sin(yaw + dyaw)
        J[1,1] = (fy/z)*cos(yaw + dyaw)
        J[1,2] = (fy/z)*cos(yaw + dyaw)*(x - xo - dx) + (fy/z)*sin(yaw + dyaw)*(y - yo - dy)
        return J

    def getTotalError(self):
        tot_error = 0
        for i in range(self.mapPoint.shape[1]):
            e = self.e(self.Y[:,i],self.mapPoint[:,i])
            E = e.T * e
            tot_error = tot_error + E[0,0]
        return tot_error


        
    def solve(self):
        last_error = 0        
        for j in range(1):
            if last_error == 0:
                 last_error = self.getTotalError()
            H = np.matrix(np.zeros((self.arg.size, self.arg.size)))
            B = np.matrix(np.zeros((self.arg.size, 1)))
            for i in range(self.mapPoint.shape[1]):
                jacobian_i = self.jacobian_i(self.mapPoint[:,i]) 
                e = self.e(self.Y[:,i],self.mapPoint[:,i])
                H = H + jacobian_i.T * jacobian_i
                B = B + jacobian_i.T * e
            darg = np.linalg.solve(H, -B)
            self.arg = self.arg + darg
            tot_error = self.getTotalError()
            print tot_error
            if last_error -  tot_error < 0.00001:
                break
            last_error =  tot_error



        return self.arg



class MapPoint:
    def __init__(self): 
        self.data  = np.array([ 
          [0,0,3],
          [-2,-2,3],
          [-2,2,3],
          [2,-2,3],
          [2,2,3],
          [-1,-1,3],
          [-1,1,3],
          [1,-1,3],
          [1,1,3],],
          dtype=np.float32).T
        

class Camera:
    def __init__(self): 
        self.twc   = np.matrix([          # The location of Camera (Camera to world)
          [0],
          [0],
          [0]],dtype=np.float32)
        self.Rwc  = np.matrix([           # The pose of Camera (Camera to world)
          [1,0,0],
          [0,1,0],
          [0,0,1]],dtype=np.float32)
        self.cameraMatrix = np.matrix([   # The camera Matrix (used by projection)
          [320,0,320],
          [0,320,240],
          [0,0,1]],dtype=np.float32)
        #self.roll = 0.0
        #self.pitch = 0.0
        self.yaw = 0.0
        
    def computeProjection(self, mapPoint, Rwc, twc): 
        """ CameraMatirx * (Rwc.T * (X - twc)) """
        feature = np.array(self.cameraMatrix * (Rwc.T * np.matrix(mapPoint - twc)))
        feature = feature / feature[2,:]                 
        return feature

    def computeRotationMatrix(self, x, y, z): 
        return np.matrix([
          [cos(y)*cos(z) - sin(x)*sin(y)*sin(z), -cos(x)*sin(z), sin(y)*cos(z) + sin(x)*cos(y)*sin(z)],
          [cos(y)*sin(z) + sin(x)*sin(y)*cos(z), cos(x)*cos(z),  sin(y)*sin(z) - sin(x)*cos(y)*cos(z)],
          [-cos(x)*sin(y),                        sin(x),        cos(x)*cos(y)]],dtype=np.float32)



class Drawer:
    def __init__(self, camera, mapPoint,opt): 
        fig = plt.figure()
        self.ax1 = fig.add_subplot(221, projection='3d')
        self.ax1.set_xlabel('X Label')
        self.ax1.set_ylabel('Y Label')
        self.ax1.set_zlabel('Z Label')
        self.ax1.set_title('map point')
        self.ax2 = fig.add_subplot(223)
        self.ax2.axis([0, 640, 0, 480])
        self.ax2.set_xlabel('u Label')
        self.ax2.set_ylabel('v Label')
        self.ax2.invert_yaxis()
        self.ax2.set_title('Camera View')
        self.ax3 = fig.add_subplot(224)
        self.ax3.axis([-5, 5, -5, 5])
        self.ax3.set_xlabel('x')
        self.ax3.set_ylabel('y')
        self.ax3.set_title('SLAM')

        self.camera = camera
        self.mapPoint = mapPoint
        self.opt = opt
        self.v = np.array([[0.],[1.],[0.]])
        vc = np.dot(self.camera.Rwc, self.v)
        self.cameraPoseSHOW = self.ax1.quiver(0., 0., 0., vc[0][0], vc[1][0], vc[2][0], color='g' ,length=0.5, normalize=True)
        self.ax1.scatter(mapPoint.data[0,:], mapPoint.data[1,:], mapPoint.data[2,:], c='g', marker='o', s=20)
        feature = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        self.featureNewSHOW = self.ax2.scatter(feature[0,:], feature[1,:], s=10)
        #self.featureOldSHOW = self.ax2.scatter(feature[0,:], feature[1,:], s=10)
        fig.canvas.mpl_connect('key_press_event', self.press)
        self.twcOld = np.matrix([[0.],[0.],[0.]])
        self.yawOld = 0
        self.ax3.scatter(0., 0., c='r',s=10)

        plt.show()

    def press(self, event):
        sys.stdout.flush()
        featureOld = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        cmd = np.matrix([[0.],[0.],[0.]])
        self.cameraPoseSHOW.remove()
        self.featureNewSHOW.remove()
        #self.featureOldSHOW.remove()
        if event.key == 'left':
            cmd = np.matrix([[0.],[0.],[0.05]])
        if event.key == 'right':
            cmd = np.matrix([[0.],[0.],[-0.05]])
        if event.key == 'up':
            dt = -self.camera.Rwc * np.matrix([[0],[-0.20],[0]])
            cmd = np.matrix([[dt[0,0]],[dt[1,0]],[0.0]])
        if event.key == 'down':
            dt = -self.camera.Rwc* np.matrix([[0],[0.20],[0]])
            cmd = np.matrix([[dt[0,0]],[dt[1,0]],[0.0]])

        self.camera.yaw = self.camera.yaw + cmd[2,0]
        self.camera.Rwc = self.camera.computeRotationMatrix(0, 0, self.camera.yaw)
        self.camera.twc = self.camera.twc + np.matrix([[cmd[0,0]],[cmd[1,0]],[0]])
        vc = np.dot(self.camera.Rwc, self.v)
        self.cameraPoseSHOW = self.ax1.quiver(self.camera.twc[0, 0], self.camera.twc[1, 0], self.camera.twc[2, 0], vc[0][0], vc[1][0], vc[2][0], color='g' ,length=0.5, normalize=True)
        feature = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        self.featureNewSHOW = self.ax2.scatter(feature[0,:], feature[1,:], c='r',s=10)
        #self.featureOldSHOW = self.ax2.scatter(featureOld[0,:], featureOld[1,:], c='b',s=10)
        #np.matrix([[0.],[0.],[0.]])
        self.opt.set(feature, self.mapPoint.data, cmd, 320, 320, 320, 240, self.yawOld, self.twcOld)
        newArg = self.opt.solve()
        print newArg
        self.yawOld = self.yawOld + newArg[2,0]
        self.twcOld[0,0] = self.twcOld[0,0] + newArg[0,0]
        self.twcOld[1,0] = self.twcOld[1,0] + newArg[1,0]
        slam_path = self.ax3.scatter(self.twcOld[0,0], self.twcOld[1,0],  c='r', s=10)
        real_path = self.ax3.scatter(self.camera.twc[0,0], self.camera.twc[1,0],  c='b', s=10)
        #self.ax3.legend()
        plt.legend((slam_path, real_path),
           ('slam_path', 'real_path'),
           scatterpoints=1,
           loc='lower left',
           ncol=3,
           fontsize=8)
        #self.ax3.scatter(self.twcOld, self.twcOld,  c='r', s=10)

        plt.draw()
        


mapPoint = MapPoint()
camera = Camera()
opt = Optimizer()
Drawer(camera, mapPoint,opt)

