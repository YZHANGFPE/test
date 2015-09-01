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
        self.gripper = Gripper("right")
        self.gripper.calibrate()
        self.gripper.set_moving_force(0.1)
        rospy.sleep(0.5)
        self.gripper.set_holding_force(0.1)
        rospy.sleep(0.5)
        self.busy = False
        self.gripper_distance = 100
        self.subscribe_gripper()
        self.robotx = -1
        self.roboty = -1
        self.framenumber = 0
        self.history = np.arange(0,20)*-1
        self.newPosition = True
        self.bowlcamera = None
        self.kin = baxter_kinematics('right')
        self.J = self.kin.jacobian()
        self.J_inv = self.kin.jacobian_pseudo_inverse()
        self.jointVelocity = np.asarray([1,2,3,4,5,6,7],np.float32)
        self.control_arm = Limb("right")
        self.control_joint_names = self.control_arm.joint_names()
        self.dx = 0
        self.dy = 0
        self.distance = 1000
        self.finish = False
        self.found = 0
        ifr = rospy.Subscriber("/robot/range/right_hand_range/state", Range, self._read_distance)


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
 
    def start_left_hand_vision(self):
        
        resp = self.vision_request_left('FindCup', 1)
        if resp.responseID != 0:
            print "Vision Request to Find Cup Failed: %d" % resp.responseID
            return

    def moveToCup(self):  
        rospy.sleep(1)
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

    def PID(self):

        Kp = 1.2E-5
        vy = -Kp*self.dx
        vx = -Kp*self.dy
        return vx,vy

    def list_to_dic(self,ls):
        cmd = {}
        for idx, name in enumerate(self.control_joint_names):
            v = ls.item(idx)
            cmd[name] = v 
        return cmd

    def track(self, initial_pose, hdr, arm):

        # Position Control #
        predict_pose = initial_pose

        while self.finish != True:

            resp = self.vision_request_right(0, 1)
            print resp
            self.found = resp.response
            self.dx = resp.robotx
            self.dy = resp.roboty
            vx, vy = self.PID()
            
            if self.found == 1:
                if self.distance > 0.13:
                    solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin, target_dx = (1-abs(vy)*10)*0.0005,target_dy = vy)
                    #2D traking#
                    #solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin, target_dy = vy, target_dz = vx)
                    
                elif self.distance <= 0.13 and self.distance > 0.06:
                    solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, self.kin, target_dx = 0.001, target_dy = 0)
                    
                else:
                    self.finish = True

        # Velocity Control #

        # v_end = np.asarray([(1-((abs(vx)+abs(vy))*5))*0.03,vy,vx,0,0,0],np.float32)
        
        # v_end = np.asarray([0,vy,vx,0,0,0],np.float32)
        # v_joint = np.dot(self.J_inv,v_end)
        # cmd = self.list_to_dic(v_joint)
        # self.finish = False
        # if self.found == 1:
        #     if self.distance > 0.1:
        #         self.control_arm.set_joint_velocities(cmd) 
        #         #rospy.sleep(0.02)
        #     else:
        #         self.finish = True


    def rotate(self,target_rotate_angle):

        dtheta = 100
        current_angles = self.control_arm.joint_angles()
        current_angle = current_angles['right_w2']
        target_angle = current_angle + target_rotate_angle
        cmd = current_angles
        while abs(dtheta) > 0.01:
            current_angles = self.control_arm.joint_angles()
            current_angle = current_angles['right_w2']
            dtheta = target_angle - current_angle
            cmd['right_w2'] = current_angle + dtheta/10.0
            self.control_arm.set_joint_positions(cmd)
            rospy.sleep(0.01)

    def pour(self,target_rotate_angle):

        self.rotate(target_rotate_angle)
        rospy.sleep(2)
        self.rotate(-target_rotate_angle)

    def subscribe_gripper(self):
        gripper_msg = rospy.Subscriber('robot/end_effector/right_gripper/state', EndEffectorState, self.gripper_callback)

    def gripper_callback(self,msg):
        self.gripper_distance = msg.position
        
    def errorhandle(self):
        speak("Please give me the tool")
        while (self.gripper_distance < 5):
            self.gripper.open()
            rospy.sleep(3)
            self.gripper.close()
            rospy.sleep(0.5)

    def clean_shutdown(self):
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
                
def set_joints( target_angles_right, target_angles_left):

    right = Limb("right")
    left = Limb("left")
    
    reach_right = False
    reach_left = False
    

    while not reach_right or not reach_left:

            right.set_joint_positions(target_angles_right)
            left.set_joint_positions(target_angles_left)
            current_angles_right = right.joint_angles()
            current_angles_left = left.joint_angles()

            

            for k, v in current_angles_right.iteritems():
                if abs(target_angles_right[k] - v) > 0.01:
                    reach_right = False
                    break
                reach_right = True

            for k, v in current_angles_left.iteritems():
                if abs(target_angles_left[k] - v) > 0.01:
                    reach_left = False
                    break
                reach_left = True

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

def ik(pose):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
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
        arm = Limb("right")
        arm.set_joint_positions(limb_joints)
        return 1
        #rospy.sleep(0.05)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0

def ik_pykdl(arm, kin, pose):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)
    cmd = {
        'right_s0': joint_angles[0],
        'right_s1': joint_angles[1],
        'right_e0': joint_angles[2],
        'right_e1': joint_angles[3],
        'right_w0': joint_angles[4],
        'right_w1': joint_angles[5],
        'right_w2': joint_angles[6],
    }

    arm.set_joint_positions(cmd)
    
def ik_move(hdr, arm, target_dx = None, target_dy = None, target_dz = None, x = None, y = None, z = None):

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

    while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and solution_found == 1:
    
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
        vx = dx
        vy = dy
        vz = dz
        #print dx, dy, dz
        new_pose = update_current_pose(current_pose,vx,vy,vz)
        solution_found = ik(new_pose)

    return solution_found
        
def ik_move_one_step(initial_pose, predict_pose, hdr, arm, kin, target_dx = None, target_dy = None, target_dz = None):



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
    solution_found = ik_pykdl(arm, kin, new_pose)
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

def main():

    roscpp_initialize(sys.argv)
    rospy.init_node('demo', anonymous=True)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    arm = Limb("right")

    # Move to initail position 
    joints_initialize()
    tracker=track()
    rospy.on_shutdown(tracker.clean_shutdown)
    # Track the pitcher 
    initial_pose = get_current_pose(hdr,arm)
    tracker.track(initial_pose, hdr, arm)
    # Pick up the pitcher
    tracker.gripper.close()
    rospy.sleep(0.5)
    # Move to standby position
    ik_move(hdr,arm, target_dz = 0.1)
    ik_move(hdr,arm, x = 0.65 , y = -0.3)
    # Look for the cup and perform the task
    tracker.start_left_hand_vision()
    while not rospy.is_shutdown():
        tracker.moveToCup()
     
if __name__=='__main__':
    sys.exit(main())
    