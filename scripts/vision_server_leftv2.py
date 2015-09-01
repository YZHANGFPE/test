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

        rospy.init_node('vision_server_left_v2')
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
        cv2.namedWindow('image')
        self.np_image = np.zeros((400,640,3), np.uint8)
        cv2.imshow('image',self.np_image)
        
        s = rospy.Service('vision_server_left_v2', VisionVertical, self.handle_vision_req)
        camera_topic = '/cameras/left_hand_camera/image'
        self.right_camera = rospy.Subscriber(camera_topic, Image, self._on_camera)
        print "\nReady to use right hand vision server\n" 

        self.kin = baxter_kinematics('right')
        self.J = self.kin.jacobian()
        self.J_inv = self.kin.jacobian_pseudo_inverse()


    def _on_camera(self, data):

        self.framenumber += 1
        index = self.framenumber % 10
        cv2.namedWindow('image')
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "bgr8")
        np_image = np.asarray(cv_image)
        if self.request == 1 :
            np_image, mask = self.image_process_cap(np_image)
        elif self.request == 2:
            np_image, mask = self.image_process_cap(np_image)
        elif self.request == 3:
            np_image, mask = self.image_process_cap(np_image)
        elif self.request == 4:
            np_image, mask = self.image_process_cap(np_image)
        else:
            self.dx = 0
            self.dy = 0
            self.close = 0
            np_image, mask = self.image_process_cap(np_image)
        cv2.imshow('image',np_image)
        cv2.waitKey(1)


    def image_process_cap(self,img):

        # tomato
        min_r = 103
        max_r = 133
        min_g = 58 
        max_g = 109
        min_b = 0
        max_b = 51

        centerx = 420
        centery = 120

        min_color = (min_r, min_g, min_b)
        max_color = (max_r, max_g, max_b)

        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, min_color, max_color)
        

        #result = cv2.bitwise_and(img, img, mask = mask)
        mask_bool = np.asarray(mask, bool)
        label_img = morphology.remove_small_objects(mask_bool, 800, connectivity = 1)
        objects = measure.regionprops(label_img)

        if objects != []:
            self.found = 1
            target = objects[0]
            box = target.bbox
            cv2.rectangle(img,(box[1],box[0]),(box[3],box[2]),(0,255,0),3)
            dx_pixel=int((box[1]))-centerx
            dy_pixel=int((box[0]+box[2])/2)-centery
            #angle = target.orientation           
            cv2.circle(img,(int((box[1] + box[3])/2),int((box[2]))),10,(0,0,255),-1)
            cv2.circle(img,(int(centerx),int(centery)),10,(0,255,0),-1)
                       
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
        resp = VisionVerticalResponse(self.request, self.found, self.dx, self.dy)
        print resp
        
        return resp

    def clean_shutdown(self):
        print "Server finished"
        rospy.signal_shutdown("Done")
        sys.exit()


if __name__=='__main__':
    
    vision = vision_server()
    rospy.spin()
