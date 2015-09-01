#!/usr/bin/env python
import sys
import rospy
import baxter_external_devices
import argparse
from baxter_interface import CameraController, Gripper, Limb
from sensor_msgs.msg import Image, Range
import cv_bridge
import cv2
import numpy as np
import math
from skimage import segmentation, measure, morphology
import matplotlib.pyplot as plt
import copy
import pyaudio
import pyttsx
import time
import tf

import pygst
import gst

from arcbaxter.srv import *
from baxter_pykdl import baxter_kinematics


def nothing(x):
    nothing

def homography():
    dstPoints = np.asarray([[0.55,-0.5],[0.8,-0.5],[0.8,0],[0.55,0]],np.float32)
    srcPoints1 = np.asarray([[482,148,1]],np.float32).T
    srcPoints = np.asarray([[482,148],[406,133],[312,190],[399,222]],np.float32)

    H = cv2.findHomography(srcPoints,dstPoints,0)[0]
    return H

class vision_server():
    
    def __init__(self):

        rospy.init_node('kinect_vision_server')
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.busy = False
        self.dx = 0
        self.dy = 0
        self.avg_dx = -1
        self.avg_dy = -1
        self.H = homography()
        self.framenumber = 0
        self.history_x = np.arange(0,10)*-1
        self.history_y = np.arange(0,10)*-1
        self.newPosition = True
        self.centerx = 230
        self.centery = 350
        self.coefx = 0.1/(526-369)
        self.coefy = 0.1/(237-90)
        self.found = 0
        self.finish = 0
        self.request = 0
        self.close = 0
        self.box = []
        self.u = 0
        self.v = 0
        self.x = 0
        self.y = 0
        self.z = 0
        
        s = rospy.Service('kinect_vision_server', Kinect, self.handle_vision_req)
        camera_topic = '/camera/rgb/image_rect_color'
        self.right_camera = rospy.Subscriber(camera_topic, Image, self._on_camera)
        self.depth_image = rospy.Subscriber('/camera/depth_registered/image_rect_raw', Image, self.depth_camera_callback)
        print "\nReady to use right hand vision server\n" 

        self.kin = baxter_kinematics('right')
        self.J = self.kin.jacobian()
        self.J_inv = self.kin.jacobian_pseudo_inverse()


    def _on_camera(self, data):

        self.framenumber += 1
        index = self.framenumber % 10
        cv2.namedWindow('rgb_image')
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "bgr8")
        np_image = np.asarray(cv_image)
        np_image, mask = self.image_process(np_image)
        cv2.imshow('rgb_image',np_image)
        print self.x, self.y, self.z
        cv2.waitKey(1)

    def depth_camera_callback(self, data):
        self.framenumber += 1
        index = self.framenumber % 10
        # cv2.namedWindow('depth_image')
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "32FC1")
        depth_image = np.asarray(cv_image)
        d =  depth_image[self.u, self.v] # in mm

        cx = 314.17
        cy = 269.95
        fx = 514.54
        fy = 529.33
        x = (self.v-cx)*d/fx
        y = (self.u-cy)*d/fy
        kinect_point = np.asarray([x/1000,y/1000,d/1000,1])
        transform = np.asarray([[ 0.0883033 , -0.53220687 , 0.84199666 , 0.08079034],[ -0.99609123 ,-0.04532477 , 0.07581502 ,-0.01867642],[ -0.00218597 ,-0.84540021 ,-0.53412893 , 0.97077381], [  0.00000000e+00 ,  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00]])
        robot_point = np.dot(transform,np.transpose(kinect_point))
        self.x = robot_point[0]
        self.y = robot_point[1]
        self.z = robot_point[2]
        #cv2.rectangle(depth_image,(self.box[1],self.box[0]),(self.box[3],self.box[2]),(0,255,0),3)
        # cv2.imshow('depth_image',depth_image)
        # cv2.waitKey(1)


    def image_process(self,img):

        # cup
        # min_r = 9
        # max_r = 28
        # min_g = 155
        # max_g = 255
        # min_b = 0
        # max_b = 255
        #ladle
        # min_r = 95
        # max_r = 105
        # min_g = 50
        # max_g = 255
        # min_b = 20
        # max_b = 255
        # duck
        min_r = 24
        max_r = 37
        min_g = 105
        max_g = 246
        min_b = 143
        max_b = 255

        centerx = 420
        centery = 120

        min_color = (min_r, min_g, min_b)
        max_color = (max_r, max_g, max_b)

        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, min_color, max_color)
        

        #result = cv2.bitwise_and(img, img, mask = mask)
        mask_bool = np.asarray(mask, bool)
        label_img = morphology.remove_small_objects(mask_bool, 100, connectivity = 1)
        objects = measure.regionprops(label_img)

        if objects != []:
            self.found = 1
            target = objects[0]
            box = target.bbox
            self.box = box
            cv2.rectangle(img,(box[1],box[0]),(box[3],box[2]),(0,255,0),3)
            dx_pixel=int((box[1]))-centerx
            dy_pixel=int((box[0]+box[2])/2)-centery
            self.u = target.centroid[0]
            self.v = target.centroid[1]
            #angle = target.orientation           
            cv2.circle(img,(int((box[1]+box[3])/2),int((box[0]+box[2])/2)),10,(0,0,255),-1)
                       
            self.dx = dx_pixel
            self.dy = dy_pixel

        else:
            self.found = 0
            self.dx = 0
            self.dy = 0


        return img, mask_bool

    

    def handle_vision_req(self, req):
        self.request = req.requestID
        if self.close == 3: self.request = -1
        resp = KinectResponse(self.request, self.found, self.x, self.y, self.z)
        
        return resp

    def clean_shutdown(self):
        print "Server finished"
        rospy.signal_shutdown("Done")
        sys.exit()


if __name__=='__main__':
    
    vision = vision_server()
    rospy.spin()
