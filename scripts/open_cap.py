#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, conversions
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_interface import CameraController, Gripper, Limb
from sensor_msgs.msg import Image, Range
import numpy as np
import math
import copy
import speech_recognition as sr
import pyaudio
import pyttsx
import time


import pygst
import gst

from baxter_pykdl import baxter_kinematics
import PyKDL

from arcbaxter.msg import *
from arcbaxter.srv import *

class track():
    
    def __init__(self):
        
        self.centerx = 365
        self.centery = 120
        self.coefx = 0.1/(526-369)
        self.coefy = 0.1/(237-90)
        self.count = 0
        self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        self.gripper_left = Gripper("left")
        self.gripper_left.calibrate()
        self.gripper_left.set_moving_force(0.01)
        rospy.sleep(0.5)
        self.gripper_left.set_holding_force(0.01)
        rospy.sleep(0.5)
        self.gripper_right = Gripper("right")
        self.gripper_right.calibrate()
        rospy.sleep(1)
        self.busy = False
        self.gripper_distance = 100
        self.subscribe_gripper()
        self.robotx = -1
        self.roboty = -1
        self.framenumber = 0
        self.history = np.arange(0,20)*-1
        self.newPosition = True
        self.bowlcamera = None
        self.kin_right = baxter_kinematics('right')
        self.kin_left = baxter_kinematics('left')
        self.J = self.kin_right.jacobian()
        self.J_inv = self.kin_right.jacobian_pseudo_inverse()
        self.jointVelocity = np.asarray([1,2,3,4,5,6,7],np.float32)
        self.control_arm = Limb("right")
        self.left_arm = Limb("left")
        self.control_joint_names = self.control_arm.joint_names()
        self.dx = 0
        self.dy = 0
        self.distance = 1000
        self.finish = False
        self.found = 0
        self.pour_x = 0
        self.pour_y = 0
        ifr = rospy.Subscriber("/robot/range/right_hand_range/state", Range, self._read_distance)
        self.joint_states = {

            'observe':{
                        'right_e0': -0.631,
                        'right_e1': 0.870,
                        'right_s0': 0.742, 
                        'right_s1': -0.6087,
                        'right_w0': 0.508,
                        'right_w1': 1.386,
                        'right_w2': -0.5538,
                    },
                    'observe_ladle':{
                        'right_e0': -0.829,
                        'right_e1': 0.831,
                        'right_s0': 0.678, 
                        'right_s1': -0.53,
                        'right_w0': 0.716,
                        'right_w1': 1.466,
                        'right_w2': -0.8099,
                    },
                    'observe_left':{
                        'left_w0': 2.761932405432129, 
                        'left_w1': -1.5700293346069336, 
                        'left_w2': -0.8893253607604981, 
                        'left_e0': -0.9805972175354004, 
                        'left_e1': 1.8300390778564455, 
                        'left_s0': 1.4783739826354982, 
                        'left_s1': -0.9503010970092775,

                    },
                    'stir':{
                        'right_e0': -0.179,
                        'right_e1': 1.403,
                        'right_s0': 0.381, 
                        'right_s1': -0.655,
                        'right_w0': 1.3,
                        'right_w1': 2.04,
                        'right_w2': 0.612,

                    },
                    'observe_vertical':{
                        'right_e0': 0.699,
                        'right_e1': 1.451,
                        'right_s0': -1.689, 
                        'right_s1': 0.516,
                        'right_w0': 0.204,
                        'right_w1': 0.935,
                        'right_w2': -2.706,
                    },
                    'observe_midpoint':{
                        'right_e0': -0.606,
                        'right_e1': 0.968,
                        'right_s0': 0.0, 
                        'right_s1': -0.645,
                        'right_w0': 0.465,
                        'right_w1': 1.343,
                        'right_w2': -0.437,
                    },
                    'dressing':{
                        'right_e0': 0.967,
                        'right_e1': 1.386,
                        'right_s0': -0.348, 
                        'right_s1': -0.155,
                        'right_w0': 0.264,
                        'right_w1': 1.521,
                        'right_w2': -2.199,
                    },
        
        }


    def _read_distance(self,data):
        self.distance = data.range

    def vision_request_right(self, controlID, requestID):
        
        try:
            
            rospy.wait_for_service('vision_server_vertical')
            vision_server_req = rospy.ServiceProxy('vision_server_vertical', VisionVertical)
            return vision_server_req(controlID, requestID)
        
        except (rospy.ServiceException,rospy.ROSInterruptException), e:
            print "Service call failed: %s" % e
            self.clean_shutdown_service()
            
    def vision_request_left(self, taskname, requestID):
        
        try:
            rospy.wait_for_service('vision_server_left')
            vision_server_req = rospy.ServiceProxy('vision_server_left', Vision)
            return vision_server_req(taskname, requestID)
        
        except (rospy.ServiceException,rospy.ROSInterruptException), e:
            print "Service call failed: %s" % e
            self.clean_shutdown_service()

    def vision_request_left_cap(self, controlID, requestID):
        
        try:
            rospy.wait_for_service('vision_server_left_v2')
            vision_server_req = rospy.ServiceProxy('vision_server_left_v2', VisionVertical)
            return vision_server_req(controlID, requestID)
        
        except (rospy.ServiceException,rospy.ROSInterruptException), e:
            print "Service call failed: %s" % e
            self.clean_shutdown_service()
 
    def start_left_hand_vision(self):
        
        resp = self.vision_request_left('FindBowl', 1)
        if resp.responseID != 0:
            print "Vision Request to Find Bowl Failed: %d" % resp.responseID
            return

    def moveToCup(self):  

        foundCup = 0
        history = VisionResponse('FindCup', -1, 0, [])

        while foundCup != 1:
            #rospy.sleep(1)
            resp = self.vision_request_left('FindCup', 0)
            print resp
            if resp != None and resp.responseID == 0 and resp.frameNum - history.frameNum > 20:
                
                
                if checkdiff(history.data, resp.data, 0.2) is True:
                    history = resp

                else:
                    if len(resp.data) == 0:                        
                        print "I can't find the cup"

                    else: 
                        foundCup = 1
                        # Correct the offset
                        x = resp.data[0] - 0.03
                        y = resp.data[1] - 0.18
        

        print "Move to cup"
        if ik_move(self.hdr,self.control_arm,  x = x , y = y) == 1:
            rospy.sleep(2)
            self.pour(-math.pi/2)
            ik_move(self.hdr,self.control_arm, x = 0.65 , y = -0.3)
        else:
            print "I can't reach the cup"
            speak("I can't reach the cup")

    def moveToBowl(self):  
        self.start_left_hand_vision()
        foundBowl = 0
        history = VisionResponse('FindBowl', -1, 0, [])

        while foundBowl != 1:
            #rospy.sleep(1)
            resp = self.vision_request_left('FindBowl', 0)
            if resp != None and resp.responseID == 0 and resp.frameNum - history.frameNum > 1:
                
                if checkdiff(history.data, resp.data, 0) is True:
                    history = resp

                else:
                    x = resp.data[0]
                    y = resp.data[1]
                    print resp
                    if len(resp.data) == 0:
                        speak("I can't reach the bowl")
                    elif y>0.15 or x > 0.65:
                        speak("Could you please move the bowl closer?")
                        rospy.sleep(2)
                    else: 
                        foundBowl = 1
                        

        

        print "Move to bowl"
        return x,y

    def PID(self):

        Kp = 0.5E-5
        vy = -Kp*self.dx
        vx = -Kp*self.dy
        return vx,vy

    def list_to_dic(self,ls):
        cmd = {}
        for idx, name in enumerate(self.control_joint_names):
            v = ls.item(idx)
            cmd[name] = v 
        return cmd

    def track(self, initial_pose, hdr, arm, id, timeout = 1200):

        # Position Control #
        predict_pose = initial_pose
        threshold_z = [-0.08,-0.11]
        obj = ["strawberry","ladle","dressing", "dressing"]
        time = 0
        failed_times = 0
        consectuive_failures = 0 
        while self.finish != True:
            current_pose = get_current_pose(self.hdr,self.control_arm)
            z = current_pose.pose.position.z
            print z
            resp = self.vision_request_right(0, id)
            print resp
            close = resp.responseID
            self.found = resp.response
            self.dx = resp.robotx
            self.dy = resp.roboty
            vx, vy = self.PID()
            time += 1
            
            if self.found == 1:
                #if self.distance > 0.10:
                consectuive_failures = 0
                if id == 3 or id == 4:
                    if close != -1 :
                       
                        solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin_right, target_dx = (1-(abs(vy)+abs(vx))*30)*0.0005,target_dy = vy )
                        if not solution_found:
                            sentence = "could you please move the " + obj[id-1] + " closer?"
                            speak(sentence)
                        #2D traking#
                        #solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin, target_dx = -vx, target_dy = -vy)
                    
                    else:
                        ik_move(hdr,arm, target_dx = 0.085,speedx = 0.3, timeout = 3)
                        
                        self.finish = True
                        self.vision_request_right(0, 0)
                else:
                    if z > threshold_z[id-1]:
                        print "I am here"
                       
                        solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin_right, target_dx = vx, target_dy = vy, target_dz = -(1-(abs(vy)+abs(vx))**2*3600)*0.0003*(id+0.2))
                        
                        if not solution_found:
                            sentence = "could you please move the " + obj[id-1] + " closer?"
                            speak(sentence)
                        #2D traking#
                        #solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin, target_dx = -vx, target_dy = -vy)
                    
                    else:
                        self.finish = True
                        self.vision_request_right(0, 0)
            else:
                if consectuive_failures > 10: # In case that the tracking may fail even though the object is there
                    consectuive_failures = 0
                    if failed_times < 3:
                        sentence = "I could not find the " + obj[id-1]
                        speak(sentence)
                        if id == 3:
                            set_joints(target_angles_right = self.joint_states['observe_vertical'])
                        elif id == 2:
                            set_joints(target_angles_right = self.joint_states['observe_ladle'])
                        else:
                            set_joints(target_angles_right = self.joint_states['observe'])
                        rospy.sleep(3)
                        failed_times += 1
                        predict_pose = initial_pose
                    else:
                        speak("I could not continue the task")
                        self.clean_shutdown()
                else:
                    consectuive_failures += 1



        self.finish = False

    def track_cap(self, initial_pose, hdr, arm, id, timeout = 1200):

        predict_pose = initial_pose

        time = 0
        while self.finish != True and not rospy.is_shutdown():
            current_pose = get_current_pose(self.hdr,self.left_arm)
            resp = self.vision_request_left_cap(0, 1)
            print resp
            close = resp.responseID
            self.found = resp.response
            self.dx = resp.robotx
            self.dy = resp.roboty
            vx, vy = self.PID()
            time += 1
            
            if self.found == 1:
                
                solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin_left, target_dx = vx, target_dy = vy, side = 'left')

        

    def velocity_control(self):

        #Velocity Control

        
        v_end = np.asarray([0,0,0,0.2,0,0],np.float32)
        v_joint = np.dot(self.J_inv,v_end)
        cmd = self.list_to_dic(v_joint)
        self.finish = False
        for i in range(100000):
            print i
            self.control_arm.set_joint_velocities(cmd) 



    def rotate(self,target_rotate_angle):

        dtheta = 100
        current_angles = self.control_arm.joint_angles()
        current_angle = current_angles['right_w2']
        target_angle = current_angle + target_rotate_angle
        cmd = current_angles
        while abs(dtheta) > 0.01 and not rospy.is_shutdown():
            current_angles = self.control_arm.joint_angles()
            current_angle = current_angles['right_w2']
            dtheta = target_angle - current_angle
            cmd['right_w2'] = current_angle + dtheta/10.0
            self.control_arm.set_joint_positions(cmd)
            rospy.sleep(0.01)

    def rotate_left(self,target_rotate_angle, speed = 0.1):

        dtheta = 100
        current_angles = self.left_arm.joint_angles()
        current_angle = current_angles['left_w2']
        target_angle = current_angle + target_rotate_angle
        cmd = current_angles
        start = rospy.get_time()
        while abs(dtheta) > 0.01 and not rospy.is_shutdown() and (rospy.get_time() - start < 5.0):
            current_angles = self.left_arm.joint_angles()
            current_angle = current_angles['left_w2']
            dtheta = target_angle - current_angle
            cmd['left_w2'] = current_angle + dtheta*speed
            self.left_arm.set_joint_positions(cmd)
            rospy.sleep(0.01)

    def open(self, close = False, rotation = 2):

        if close == True: angle = -1.9*math.pi
        else: angle = math.pi/2

        for i in range(rotation):
            self.gripper_left.close()
            rospy.sleep(0.5)
            self.rotate_left(-angle)
            rospy.sleep(0.5)
            self.gripper_left.open()
            rospy.sleep(0.5)
            self.rotate_left(angle)
            rospy.sleep(0.5)

    def pour(self,target_rotate_angle):

        self.rotate(target_rotate_angle)
        rospy.sleep(2)
        self.rotate(-target_rotate_angle)

    def pour_tomato(self):
        #self.errorhandle("tomato")
        pour_tomato_120q = [-0.03,0.54,0.84,0.0026]
        pour_tomato_90q = [0.008,0.705,0.708,0.0443]
        pour_tomato_45q = [0.015,0.902,0.423,0.085]

        initial_pose = get_current_pose(self.hdr,self.control_arm)
        upstraight_pose = copy.deepcopy(initial_pose)

        x,y = self.moveToBowl()
        upstraight_pose.pose.position.x = x + 0.1
        upstraight_pose.pose.position.y = y - 0.1

        #ik_move(self.hdr,self.control_arm, x = 0.7 , y = -0.2)
        
        ik_move_to_pose(self.control_arm,self.kin_right,upstraight_pose,timeout = 20000)
        pour_pose = copy.deepcopy(upstraight_pose)
        pour_pose.pose.position.z += 0.02
        pour_pose.pose.orientation.x = pour_tomato_45q[0]
        pour_pose.pose.orientation.y = pour_tomato_45q[1]
        pour_pose.pose.orientation.z = pour_tomato_45q[2]
        pour_pose.pose.orientation.w = pour_tomato_45q[3]
        ik_move_to_pose(self.control_arm,self.kin_right,pour_pose,timeout = 20000)
        pour_pose.pose.orientation.x = pour_tomato_120q[0]
        pour_pose.pose.orientation.y = pour_tomato_120q[1]
        pour_pose.pose.orientation.z = pour_tomato_120q[2]
        pour_pose.pose.orientation.w = pour_tomato_120q[3]
        ik_move_to_pose(self.control_arm,self.kin_right,pour_pose,timeout = 40000)
        pour_pose.pose.orientation.x = pour_tomato_45q[0]
        pour_pose.pose.orientation.y = pour_tomato_45q[1]
        pour_pose.pose.orientation.z = pour_tomato_45q[2]
        pour_pose.pose.orientation.w = pour_tomato_45q[3]
        ik_move_to_pose(self.control_arm,self.kin_right,pour_pose,timeout = 20000)
        ik_move_to_pose(self.control_arm,self.kin_right,upstraight_pose,timeout = 20000)
        ik_move_to_pose(self.control_arm,self.kin_right,initial_pose)

    def stir_salad(self,stir_joint_angles):
        self.errorhandle("ladle")
        initial_pose = get_current_pose(self.hdr,self.control_arm)
        initial_angles = Limb("right").joint_angles()
        set_joints(target_angles_right = stir_joint_angles)
        upstraight_pose = get_current_pose(self.hdr,self.control_arm)
        ready_pose = copy.deepcopy(upstraight_pose)

        x,y = self.moveToBowl()
        ready_pose.pose.position.x = x + 0.13
        ready_pose.pose.position.y = y - 0.15
        ik_move_to_pose(self.control_arm,self.kin_right,ready_pose)
        stir_pose = copy.deepcopy(ready_pose)
        stir_pose.pose.position.z -= 0.24
        ik_move_to_pose(self.control_arm,self.kin_right,stir_pose)

        initial_stir_pose = get_current_pose(self.hdr,self.control_arm)
        initial_y = initial_stir_pose.pose.position.y
        initial_x = initial_stir_pose.pose.position.x
        current_pose = copy.deepcopy(initial_stir_pose)
        for i in range(6000):
            current_pose.pose.position.y = initial_y + math.sin(i*0.004)*0.07
            current_pose.pose.position.x = initial_x - (0.07-math.cos(i*0.004)*0.07)
            ik_pykdl(self.control_arm,self.kin_right,current_pose)

        ik_move_to_pose(self.control_arm,self.kin_right,ready_pose)
        ik_move_to_pose(self.control_arm,self.kin_right,upstraight_pose)
        set_joints(target_angles_right = initial_angles)

    def pour_dressing(self,dressing_angles):
        self.errorhandle("dressing")
        initial_pose = get_current_pose(self.hdr,self.control_arm)
        initial_angles = Limb("right").joint_angles()
        # set_joints(target_angles_right = dressing_angles)
        ready_pose = get_current_pose(self.hdr,self.control_arm)
        #x,y = self.moveToBowl()
        x = self.pour_x
        y = self.pour_y
        ready_pose.pose.position.x = x 
        ready_pose.pose.position.y = y - 0.15
        ik_move_to_pose(self.control_arm,self.kin_right,ready_pose)
        self.rotate(math.pi)
        self.rotate(-math.pi)
        # initial_stir_pose = get_current_pose(self.hdr,self.control_arm)
        # initial_y = initial_stir_pose.pose.position.y
        # initial_x = initial_stir_pose.pose.position.x
        # current_pose = copy.deepcopy(initial_stir_pose)
        # for i in range(8000):
        #     current_pose.pose.position.y = initial_y + math.sin(i*0.001)*0.08
        #     current_pose.pose.position.x = initial_x + (0.08-math.cos(i*0.001)*0.08)
        #     ik_pykdl(self.control_arm,self.kin_right,current_pose)
        # self.rotate(-math.pi)
        # set_joints(target_angles_right = dressing_angles)
        # set_joints(target_angles_right = initial_angles)


    def subscribe_gripper(self):
        gripper_msg = rospy.Subscriber('robot/end_effector/right_gripper/state', EndEffectorState, self.gripper_callback)

    def gripper_callback(self,msg):
        self.gripper_distance = msg.position
        
    def errorhandle(self,obj):
        failed_times = 0
        while (self.gripper_distance < 5):
            if failed_times >=3 : 
                speak("I could not continue the task")
                self.clean_shutdown()
            else:
                sentence = "Please give me the " + obj
                speak(sentence)
                self.gripper_right.open()
                rospy.sleep(3)
                self.gripper_right.close()
                rospy.sleep(0.5)
                failed_times += 1

    def clean_shutdown(self):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        arm = Limb("right")
        # ik_move(hdr,arm, x = 0.82 , y = -0.2)
        # ik_move(hdr,arm, z = -0.136)
        # self.gripper.open()
        # rospy.sleep(0.5)
        # ik_move(hdr,arm, target_dx= -0.2)
        print "Demo finished"
        rospy.signal_shutdown("Done")
        
    def clean_shutdown_service(self):
        print "Service shut down"
        rospy.signal_shutdown("Done")
        sys.exit()

def speak(message):
    string = message
    music_stream_uri = 'http://translate.google.com/translate_tts?tl=en&q='+string
    player = gst.element_factory_make("playbin","player")
    player.set_property('uri',music_stream_uri)
    player.set_state(gst.STATE_PLAYING)
    rospy.sleep(4)

def joints_initialize():

    target_angles_right = {

        'right_s0': 1.502, #0
        'right_s1': 0.2332,#6
        'right_w0': -2.449,#4
        'right_w1': 1.710,#3
        'right_w2': 0.065,#1
        'right_e0': -0.014,#2
        'right_e1': 1.53,#5

    }

    target_angles_left = {

        'left_w0': 2.761932405432129, 
        'left_w1': -1.5700293346069336, 
        'left_w2': -0.8893253607604981, 
        'left_e0': -0.9805972175354004, 
        'left_e1': 1.8300390778564455, 
        'left_s0': 1.4783739826354982, 
        'left_s1': -0.9503010970092775,
    }

    set_joints( target_angles_right, target_angles_left)
                
def set_joints( target_angles_right = None, target_angles_left = None,timeout= 40000):

    right = Limb("right")
    left = Limb("left")
    
    if target_angles_right == None:
        reach_right = True
    else:
        reach_right = False
    

    if target_angles_left == None:
        reach_left = True
    else:
        reach_left = False
    
    time = 0

    while not reach_right or not reach_left:

            if target_angles_right: right.set_joint_positions(target_angles_right)
            if target_angles_left: left.set_joint_positions(target_angles_left)
            current_angles_right = right.joint_angles()
            current_angles_left = left.joint_angles()

            
            if reach_right == False:
                for k, v in current_angles_right.iteritems():
                    if abs(target_angles_right[k] - v) > 0.01:
                        reach_right = False
                        break
                    reach_right = True

            if reach_left == False:
                for k, v in current_angles_left.iteritems():
                    if abs(target_angles_left[k] - v) > 0.01:
                        reach_left = False
                        break
                    reach_left = True

            time+=1
            if time > timeout:
                print "Time out"
                break

def pose_initialize(hdr):
    
    initial_pose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=0.8,
                        y=-0.32,
                        z=0.25,
                    ),
                    orientation=Quaternion(
                        x=-0.007,
                        y=0.697,
                        z=-0.029,
                        w=0.716,
                    ),
                )
    )
    return initial_pose

def get_current_pose(hdr,arm,initial_pose = None):
    
    ep_position = arm.endpoint_pose()['position']
    ep_orientation = arm.endpoint_pose()['orientation']

    if initial_pose == None:
        
        current_pose = copy.deepcopy(initial_pose)
        current_pose = PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=ep_position.x,
                            y=ep_position.y,
                            z=ep_position.z,
                        ),
                        orientation=Quaternion(
                            x=ep_orientation.x,
                            y=ep_orientation.y,
                            z=ep_orientation.z,
                            w=ep_orientation.w,
                        ),
                    )
        )
    else:
        current_pose = copy.deepcopy(initial_pose)
        current_pose.pose.position.x = ep_position.x
        current_pose.pose.position.y = ep_position.y
        current_pose.pose.position.z = ep_position.z

    return current_pose

def update_current_pose(current_pose,dx,dy,dz):
    new_pose = copy.deepcopy(current_pose)
    dx = dx/1.0
    dy = dy/1.0
    dz = dz/1.0
    new_pose.pose.position.x += dx
    new_pose.pose.position.y += dy
    new_pose.pose.position.z += dz
    #print dx,dy,dz
    return new_pose

def ik(pose, side):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    if side == "left": ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 0

    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if side == "right": arm = Limb("right")
        else: arm = Limb("left")
        arm.set_joint_positions(limb_joints)
        return 1
        #rospy.sleep(0.05)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0

def ik_pykdl(arm, kin, pose, side = "right"):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)

    if joint_angles:
        cmd = {}
        if side == "right":
            cmd = {
                'right_s0': joint_angles[0],
                'right_s1': joint_angles[1],
                'right_e0': joint_angles[2],
                'right_e1': joint_angles[3],
                'right_w0': joint_angles[4],
                'right_w1': joint_angles[5],
                'right_w2': joint_angles[6],
            }
        else:
            cmd = {
            'left_s0': joint_angles[0],
            'left_s1': joint_angles[1],
            'left_e0': joint_angles[2],
            'left_e1': joint_angles[3],
            'left_w0': joint_angles[4],
            'left_w1': joint_angles[5],
            'left_w2': joint_angles[6],
            }
        arm.set_joint_positions(cmd)
        return True
    else:
        return False

def ik_move_to_pose(arm,kin,pose,timeout= 60000):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)
    if joint_angles:
        cmd = {
            'right_s0': joint_angles[0],
            'right_s1': joint_angles[1],
            'right_e0': joint_angles[2],
            'right_e1': joint_angles[3],
            'right_w0': joint_angles[4],
            'right_w1': joint_angles[5],
            'right_w2': joint_angles[6],
        }

        set_joints(target_angles_right = cmd,timeout = timeout)
    
def ik_move(hdr, arm, target_dx = None, target_dy = None, target_dz = None, x = None, y = None, z = None, timeout= 10,speedx = 1.0,speedy = 1.0,speedz = 1.0, side = "right"):

    initial_pose = get_current_pose(hdr,arm)
    target_x = initial_pose.pose.position.x
    target_y = initial_pose.pose.position.y
    target_z = initial_pose.pose.position.z
    
    if target_dx != None: target_x += target_dx
    if target_dy != None: target_y += target_dy
    if target_dz != None: target_z += target_dz

    if x != None: target_x = x
    if y != None: target_y = y
    if z != None: target_z = z

    dx = 100
    dy = 100
    dz = 100

    solution_found = 1

    start = rospy.get_time()
    while (abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dz) > 0.001) and solution_found == 1 and (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
    
    #while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and i < 5000:
        
        current_pose = get_current_pose(hdr,arm,initial_pose)
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        #vx = np.sign(dx)*min(0.02,abs(dx))
        #vy = np.sign(dy)*min(0.02,abs(dy))
        #vz = np.sign(dz)*min(0.02,abs(dz))
        vx = dx*speedx
        vy = dy*speedy
        vz = dz*speedz
        #print dx, dy, dz
        new_pose = update_current_pose(current_pose,vx,vy,vz)
        solution_found = ik(new_pose, side)

    return solution_found
        
def ik_move_one_step(initial_pose, predict_pose, hdr, arm, kin, target_dx = None, target_dy = None, target_dz = None, side = "right"):



    current_pose = get_current_pose(hdr,arm,initial_pose)

    if target_dx == None: 
        target_dx = initial_pose.pose.position.x - current_pose.pose.position.x
    else: 
        target_dx = target_dx + predict_pose.pose.position.x - current_pose.pose.position.x
    if target_dy == None: 
        target_dy = initial_pose.pose.position.y - current_pose.pose.position.y
    else: 
        target_dy = target_dy + predict_pose.pose.position.y - current_pose.pose.position.y
    if target_dz == None: 
        target_dz = initial_pose.pose.position.z - current_pose.pose.position.z
    else:
        target_dz = target_dz + predict_pose.pose.position.z - current_pose.pose.position.z


    new_pose = update_current_pose(current_pose,target_dx,target_dy,target_dz)
    solution_found = ik_pykdl(arm, kin, new_pose, side = side)
    return solution_found, new_pose

def checkdiff(a,b, threshold):
    if len(a) != len(b) :
        return True
    elif len(a) == 0:
        print "empty list"
        return False
    else:
        for i in range(len(a)):
            if abs(a[i]-b[i]) > threshold  :
                return True

    return False

def speak(message):
    string = message
    music_stream_uri = 'http://translate.google.com/translate_tts?tl=en&q='+string
    player = gst.element_factory_make("playbin","player")
    player.set_property('uri',music_stream_uri)
    player.set_state(gst.STATE_PLAYING)

    rospy.sleep(4)

def move_to_pose(left_pose = None, right_pose = None, timeout = 2.0):
    start = rospy.get_time()
    while not rospy.is_shutdown() and (rospy.get_time() - start) < timeout:
        if left_pose != None : ik(left_pose,"left")
        if right_pose != None :ik(right_pose, "right")


def main():

    joint_states = {
        # 'observe':{
        #     'right_e0': -0.365,
        #     'right_e1': 1.061,
        #     'right_s0': 0.920, 
        #     'right_s1': -0.539,
        #     'right_w0': 0.350,
        #     'right_w1': 1.105,
        #     'right_w2': -0.221,
        # },
        'observe':{
            'right_e0': -0.631,
            'right_e1': 0.870,
            'right_s0': 0.742, 
            'right_s1': -0.6087,
            'right_w0': 0.508,
            'right_w1': 1.386,
            'right_w2': -0.5538,
        },
        'observe_ladle':{
            'right_e0': -0.829,
            'right_e1': 0.831,
            'right_s0': 0.678, 
            'right_s1': -0.53,
            'right_w0': 0.716,
            'right_w1': 1.466,
            'right_w2': -0.8099,
        },
        'observe_left':{
            'left_w0': 2.761932405432129, 
            'left_w1': -1.5700293346069336, 
            'left_w2': -0.8893253607604981, 
            'left_e0': -0.9805972175354004, 
            'left_e1': 1.8300390778564455, 
            'left_s0': 1.4783739826354982, 
            'left_s1': -0.9503010970092775,

        },
        'stir':{
            'right_e0': -0.179,
            'right_e1': 1.403,
            'right_s0': 0.381, 
            'right_s1': -0.655,
            'right_w0': 1.3,
            'right_w1': 2.04,
            'right_w2': 0.612,

        },
        'observe_vertical':{
            'right_e0': 0.699,
            'right_e1': 1.451,
            'right_s0': -1.689, 
            'right_s1': 0.516,
            'right_w0': 0.204,
            'right_w1': 0.935,
            'right_w2': -2.706,
        },
        'observe_midpoint':{
            'right_e0': -0.606,
            'right_e1': 0.968,
            'right_s0': 0.0, 
            'right_s1': -0.645,
            'right_w0': 0.465,
            'right_w1': 1.343,
            'right_w2': -0.437,
        },
        'dressing':{
            'right_e0': 0.967,
            'right_e1': 1.386,
            'right_s0': -0.348, 
            'right_s1': -0.155,
            'right_w0': 0.264,
            'right_w1': 1.521,
            'right_w2': -2.199,
        },
        'opening_right':{
            'right_e0': 0.40075,
            'right_e1': 0.82145,
            'right_s0': 0.86133, 
            'right_s1': -0.22281,
            'right_w0': 1.31654,
            'right_w1': 1.21338,
            'right_w2': 0.98942,
        },
        'opening_left':{

            'left_e0': -0.2197, 
            'left_e1': 0.8583, 
            'left_s0': -0.6493, 
            'left_s1': -0.9410,
            'left_w0': 0.11888, 
            'left_w1': 1.611833, 
            'left_w2': -0.19788, 

        },
    }



    roscpp_initialize(sys.argv)
    rospy.init_node('open_cap_demo', anonymous=True)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    left_arm = Limb("left")
    right_arm = Limb("right")

    set_joints(target_angles_right = joint_states['observe_midpoint'],target_angles_left = joint_states['observe_left'],timeout= 100000)
    tracker=track()
    rospy.on_shutdown(tracker.clean_shutdown)

    #pour tomato
    set_joints(target_angles_right = joint_states['observe'])
    initial_pose = get_current_pose(hdr,right_arm)
    speak("I am looking for the strawberry")
    tracker.track(initial_pose, hdr, right_arm, id = 1)
    tracker.gripper_right.close()
    rospy.sleep(2)
    ik_move(hdr,right_arm, target_dz = 0.25, timeout = 2.5)
    tracker.pour_tomato()
    ik_move(hdr,right_arm, target_dz = -0.22, speedz = 0.2, timeout = 3)
    tracker.gripper_right.open()
    rospy.sleep(0.5)
    ik_move(hdr,right_arm, target_dy = -0.05,target_dz = 0.20, timeout = 2)

    # Dressing
    set_joints(target_angles_right = joint_states['observe_midpoint'],timeout= 50000)
    speak("I am looking for the dressing")
    set_joints(target_angles_right = joint_states['observe_vertical'],timeout= 400000)
    

    initial_pose = get_current_pose(hdr,right_arm)
    tracker.track(initial_pose, hdr, right_arm, id = 4)
    tracker.gripper_right.close()
    rospy.sleep(1)
    ik_move(hdr,right_arm, side = "right", target_dz = 0.25, timeout = 3)
    start_pose = get_current_pose(hdr, right_arm)
    tracker.pour_x, tracker.pour_y = tracker.moveToBowl()
    tracker.vision_request_left('FindBowl', 2)

    # open_cap
    rospy.sleep(0.1)
    left_pose = PoseStamped()
    left_pose.header = hdr
    left_pose.pose.position.x = 0.70
    left_pose.pose.position.y = 0.263
    left_pose.pose.position.z = 0.389
    left_pose.pose.orientation.x = 0
    left_pose.pose.orientation.y = 1
    left_pose.pose.orientation.z = 0
    left_pose.pose.orientation.w = 0

    right_pose = PoseStamped()
    right_pose.header = hdr
    right_pose.pose.position.x = 0.705
    right_pose.pose.position.y = 0.21
    right_pose.pose.position.z = 0.19
    right_pose.pose.orientation.x = -0.5
    right_pose.pose.orientation.y = 0.5
    right_pose.pose.orientation.z = 0.5
    right_pose.pose.orientation.w = 0.5


    move_to_pose(left_pose = left_pose, right_pose = right_pose, timeout = 8)
    rospy.sleep(2)


    dist = 0.058
    ik_move(hdr,left_arm, target_dz = -dist, side = "left", speedz = 0.1)
    tracker.open()
    tracker.rotate_left(-math.pi/2)
    rospy.sleep(0.5)
    tracker.gripper_left.close()
    rospy.sleep(1)
    ik_move(hdr,left_arm, target_dz = +dist, side = "left", speedz = 1)
    #rospy.sleep(2)

    tracker.pour_dressing(joint_states['dressing'])

    move_to_pose( right_pose = right_pose, timeout = 6)
    ik_move(hdr,left_arm, target_dz = -(dist+0.02), side = "left", speedz = 0.1, timeout = 7)
    tracker.open(close = True, rotation = 1)

    ik_move(hdr,left_arm, target_dz = +dist + 0.02, side = "left", speedz = 0.1, timeout = 4)
    rospy.sleep(2)

    set_joints(target_angles_right = joint_states['dressing'],timeout= 400000)
    move_to_pose(right_pose = start_pose, timeout = 5)


    ik_move(hdr,right_arm, side = "right", target_dz = -0.255, timeout = 3)
    tracker.gripper_right.open()
    rospy.sleep(1)
    ik_move(hdr,right_arm, side = "right", x = 0.33, speedx = 0.1, timeout = 4)
    set_joints(target_angles_right = joint_states['observe_midpoint'],target_angles_left = joint_states['observe_left'])

    # stir salad
    set_joints(target_angles_right = joint_states['observe_ladle'])
    speak("I am looking for the ladle")
    initial_pose = get_current_pose(hdr,right_arm)
    tracker.track(initial_pose, hdr, right_arm, id = 2)
    tracker.gripper_right.close()
    rospy.sleep(2)
    ik_move(hdr,right_arm, target_dz = 0.23, timeout = 2)
    tracker.stir_salad(joint_states['stir'])
    ik_move(hdr,right_arm, target_dz = -0.23, speedz = 0.2, timeout = 4)
    tracker.gripper_right.open()
    rospy.sleep(0.5)
    ik_move(hdr,right_arm, target_dz = 0.25, timeout = 2)
    speak("I am done with the task")


    

     
if __name__=='__main__':
    sys.exit(main())
    