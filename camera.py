#!/usr/bin/env python
# coding:utf-8
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from math import *
#from sys.float_info import epsilon
epsilon = sys.float_info.epsilon


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
          dtype=np.float32)
        

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
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def computeProjection(self, mapPoint, Rwc, twc): 
        """ CameraMatirx * (Rcw * X + tcw) """
        feature = np.array(self.cameraMatrix * (Rwc.T * np.matrix(mapPoint - twc.T).T))
        feature = feature / feature[2,:]                 
        return feature

    def computeRotationMatrix(self, x, y, z): 
        return np.matrix([
          [cos(y)*cos(z) - sin(x)*sin(y)*sin(z), -cos(x)*sin(z), sin(y)*cos(z) + sin(x)*cos(y)*sin(z)],
          [cos(y)*sin(z) + sin(x)*sin(y)*cos(z), cos(x)*cos(z),  sin(y)*sin(z) - sin(x)*cos(y)*cos(z)],
          [-cos(x)*sin(y),                        sin(x),        cos(x)*cos(y)]],dtype=np.float32)



class Drawer:
    def __init__(self, camera, mapPoint): 
        fig = plt.figure()
        self.ax1 = fig.add_subplot(211, projection='3d')
        self.ax1.set_xlabel('X Label')
        self.ax1.set_ylabel('Y Label')
        self.ax1.set_zlabel('Z Label')
        self.ax1.set_title('map point')
        self.ax2 = fig.add_subplot(212)
        self.ax2.axis([0, 640, 0, 480])
        self.ax2.set_xlabel('u Label')
        self.ax2.set_ylabel('v Label')
        self.ax2.invert_yaxis()
        self.ax2.set_title('Camera View')
        self.camera = camera
        self.mapPoint = mapPoint
        self.cameraPoseSHOW = self.ax1.scatter(0, 0, 0, c='b', marker='^', s=50)
        self.ax1.scatter(mapPoint.data[:,0], mapPoint.data[:,1], mapPoint.data[:,2], c='g', marker='o', s=20)
        feature = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        self.featureNewSHOW = self.ax2.scatter(feature[0,:], feature[1,:])
        self.featureOldSHOW = self.ax2.scatter(feature[0,:], feature[1,:])
        fig.canvas.mpl_connect('key_press_event', self.press)
        plt.show()

    def press(self, event):
        sys.stdout.flush()
        featureOld = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        self.cameraPoseSHOW.remove()
        self.featureNewSHOW.remove()
        self.featureOldSHOW.remove()
        if event.key == 'left':
            self.camera.yaw = self.camera.yaw + 0.05
            self.camera.Rwc = self.camera.computeRotationMatrix(self.camera.roll, self.camera.pitch, self.camera.yaw)
        if event.key == 'right':
            self.camera.yaw = self.camera.yaw - 0.05
            self.camera.Rwc = self.camera.computeRotationMatrix(self.camera.roll, self.camera.pitch, self.camera.yaw)
        if event.key == 'up':
            tcw = -self.camera.Rwc.T * self.camera.twc
            tcw = tcw + np.matrix([[0],[0.2],[0]])
            self.camera.twc = -self.camera.Rwc * tcw
            self.camera.twc = self.camera.twc + np.matrix([[0],[0.02],[0]])
        if event.key == 'down':
            tcw = -self.camera.Rwc.T * self.camera.twc
            tcw = tcw + np.matrix([[0],[-0.2],[0]])
            self.camera.twc = -self.camera.Rwc * tcw
        self.cameraPoseSHOW = self.ax1.scatter(self.camera.twc[0], self.camera.twc[1], self.camera.twc[2], c='b', marker='^', s=50)
        feature = self.camera.computeProjection(self.mapPoint.data, self.camera.Rwc, self.camera.twc)
        self.featureNewSHOW = self.ax2.scatter(feature[0,:], feature[1,:], c='r')
        self.featureOldSHOW = self.ax2.scatter(featureOld[0,:], featureOld[1,:], c='b')
        plt.draw()
        


mapPoint = MapPoint()
camera = Camera()
Drawer(camera, mapPoint)

