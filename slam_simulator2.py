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
        
    def set(self, Y, mapPoint, arg, init_arg, fx, fy, cx, cy, yaw, twc):
        self.mapPoint = mapPoint
        self.Y = Y #image
        self.arg = arg #x, y, theta (optimization object)
        self.fx = fx #camera param
        self.fy = fy #camera param 
        self.cx = cx #camera param
        self.cy = cy #camera param
        self.yaw = yaw #camera yaw
        self.twc = twc #camera pose
        self.init_arg = init_arg #x, y, theta 
        self.k = 100.#SLAM結果はProjection Errorを重視
        #self.k = 10000.#SLAM結果はOdometry Errorを重視

    def e(self,Y_i,mapPoint):
        """ This function defined the error
            Be Use for compute the H and b
        """
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
        init_dyaw = self.init_arg[2,0]
        init_dx = self.init_arg[0,0]
        init_dy = self.init_arg[1,0]

        return np.matrix([[u - (fx/z)*(cos(yaw - dyaw)*(x - xo) + sin(yaw - dyaw)*(y - yo) + dx) - cx], # Projection error 
                          [v - (fy/z)*(-sin(yaw - dyaw)*(x - xo) + cos(yaw - dyaw)*(y - yo) + dy) - cy],# Projection error 
                          [self.k*(init_dx - dx)],# odometry error 
                          [self.k*(init_dy - dy)],# odometry error 
                          [self.k*(init_dyaw - dyaw)]# odometry error 
                          ])

    def jacobian_i(self, mapPoint):
        """ This function defined the jacobian
            Be Use for compute H and b
        """
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
        J = np.matrix(np.zeros((5, 3)))
        J[0,0] = -(fx/z)
        J[0,1] = 0
        J[0,2] = -(fx/z)*sin(yaw - dyaw)*(x - xo) + (fx/z)*cos(yaw - dyaw)*(y - yo)
        J[1,0] = 0
        J[1,1] = -(fy/z)
        J[1,2] = -(fy/z)*cos(yaw - dyaw)*(x - xo) - (fy/z)*sin(yaw - dyaw)*(y - yo)
        J[2,0] = -1*self.k
        J[3,1] = -1*self.k
        J[4,2] = -1*self.k

        return J

    def getTotalError(self):
        """ Compute the total error
        """
        tot_error = 0
        for i in range(self.mapPoint.shape[1]):
            e = self.e(self.Y[:,i],self.mapPoint[:,i])
            E = e.T * e
            tot_error = tot_error + E[0,0]
        return tot_error


        
    def solve(self):
        """ A g2o implementation in Python
        """

        last_error = 0

        for j in range(10):
            if last_error == 0:
                 last_error = self.getTotalError()
                 print '---------------'
                 print 'Total error:', last_error
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
            print 'Total error:', tot_error
            if last_error -  tot_error < 0.00001:
                print 'The error is converge.'
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
        self.odom_noise  = np.matrix([[0.],[0.],[0.]])

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
        self.slamPose = np.matrix([[0.],[0.],[0.]])
        self.yawOld = 0
        self.ax3.scatter(0., 0., c='r',s=10)

        plt.show()

    def press(self, event):
        sys.stdout.flush()
        featureOld = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        cmd = np.matrix([[0.],[0.],[0.]])
        self.cameraPoseSHOW.remove()
        self.featureNewSHOW.remove()
        if event.key == 'left':
            cmd = np.matrix([[0.],[0.],[-0.05]])
        if event.key == 'right':
            cmd = np.matrix([[0.],[0.],[+0.05]])
        if event.key == 'up':
            cmd = np.matrix([[0],[-0.20],[0]])
        if event.key == 'down':
            cmd =np.matrix([[0],[0.20],[0]])

        self.camera.yaw = self.camera.yaw - cmd[2,0]
        self.camera.Rwc = self.camera.computeRotationMatrix(0, 0, self.camera.yaw)
        self.camera.twc = self.camera.twc - self.camera.Rwc * np.matrix([[cmd[0,0]],[cmd[1,0]],[0]])
        vc = np.dot(self.camera.Rwc, self.v)
        self.cameraPoseSHOW = self.ax1.quiver(self.camera.twc[0, 0], self.camera.twc[1, 0], self.camera.twc[2, 0], vc[0][0], vc[1][0], vc[2][0], color='g' ,length=0.5, normalize=True)
        feature = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        #Add noise for image
        feature_noise = feature + np.random.normal(0, 30, self.mapPoint.data.size).reshape(feature.shape)
        self.featureNewSHOW = self.ax2.scatter(feature_noise[0,:], feature_noise[1,:], c='r',s=10)
        #Add noise for odom
        #cmd_noise = cmd + np.random.normal(0, 0.03, cmd.size).reshape(cmd.shape)
        cmd_noise = cmd + np.matrix([[0.0],[0.0],[(np.random.rand()-0.5)/20]])
        newYaw = self.odom_noise[2,0] - cmd_noise[2,0]
        Rwc = self.camera.computeRotationMatrix(0, 0, newYaw)
        self.odom_noise = self.odom_noise - Rwc * np.matrix([[cmd_noise[0,0]],[cmd_noise[1,0]],[0]])
        self.odom_noise[2,0] = newYaw

        #Add noise for odom
        self.opt.set(feature_noise, self.mapPoint.data, np.matrix([[0.],[0.],[0.]]), cmd_noise, 320, 320, 320, 240, self.slamPose[2,0], self.slamPose)
        delta = self.opt.solve()
        newYaw = self.slamPose[2,0] - delta[2,0]
        Rwc = self.camera.computeRotationMatrix(0, 0, newYaw)
        self.slamPose = self.slamPose - Rwc * np.matrix([[delta[0,0]],[delta[1,0]],[0]])
        self.slamPose[2,0] = newYaw

        odom_path = self.ax3.scatter(self.odom_noise[0,0], self.odom_noise[1,0],  c='g', s=1)
        slam_path = self.ax3.scatter(self.slamPose[0,0], self.slamPose[1,0],  c='r', s=1)
        real_path = self.ax3.scatter(self.camera.twc[0,0], self.camera.twc[1,0],  c='b', s=1)

        plt.legend((odom_path, slam_path, real_path),
           ('odom_path','slam_path', 'real_path'),
           scatterpoints=1,
           loc='lower left',
           ncol=3,
           fontsize=8)
        plt.draw()

mapPoint = MapPoint()
camera = Camera()
opt = Optimizer()
Drawer(camera, mapPoint,opt)