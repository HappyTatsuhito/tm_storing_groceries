#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import tf
import time
import sys
import std_srvs.srv
import math
import actionlib
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool, Int8
from mikata_arm_msgs.msg import dxl_double

class StoringGroceries:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place',String,queue_size=1)#場所記憶
        self.navigation_command_pub = rospy.Publisher('/navigation/command_place',String,queue_size=1)#指定場所への移動
        self.object_list_req_pub = rospy.Publisher('/object/list_req',Bool,queue_size=1)#objectのリストをもらう
        self.object_grasp_req_pub = rospy.Publisher('/object/grasp_req',String,queue_size=10)#manipulationの開始
        self.changing_pose_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)#manipulateしたあとの変形
        self.object_place_req_pub = rospy.Publisher('/object/place_req',Bool,queue_size=1)#objectを置く
        self.arm_pub = rospy.Publisher('/dxl/goal_position',dxl_double,queue_size=10)#モータの制御

        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.getLaserCB)
        self.object_list_res_sub = rospy.Subscriber('/object/list_res',String,self.getObjectListCB)
        self.grasp_res_sub = rospy.Subscriber('/object/grasp_res',Bool,self.getObjectGraspResultCB)
        self.object_place_res_sub = rospy.Subscriber('/object/place_res',Bool,self.getObjectPlaceCB)
        self.navigation_res_sub = rospy.Subscriber('/navigation/result',Bool,self.getNavigationResultCB)#移動が完了したかの確認

        self.min_laser_dist = 999.9
        self.front_laser_dist = 999.9
        self.object_list = []
        self.object_list_flg = False
        self.object_grasp_result_flg = False
        self.object_num = -1
        self.object_place_flg = False
        self.navigation_result_flg = False
        self.m4_angle = dxl_double()
        self.m4_angle.id = [4]
        self.m5_angle = dxl_double()
        self.m5_angle.id = [5]
        self.twist_cmd = Twist()

    def getLaserCB(self,laser_scan):
        self.laser_dist = laser_scan.ranges
        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]

    def getObjectListCB(self,result_msg):
        self.object_list = result_msg.data.split(' ')
        self.object_list[-1:] = []
        print self.object_list
        self.object_num = len(self.object_list)
        self.object_list_flg = True
        
    def getObjectGraspResultCB(self,result_msg):
        self.object_grasp_result_flg = True

    def getObjectPlaceCB(self,result_msg):
        self.object_place_flg = True

    def getNavigationResultCB(self,result):
        self.navigation_result_flg = True

    def speak(self,sentence):
        print sentence
        voice_cmd = '/usr/bin/picospeaker %s' %sentence
        subprocess.call(voice_cmd.strip().split(' '))
        
    def inspectCupboard(self):#---------------------------------state 0
        print 'State : 0'
        self.m5_angle.data = 0.8
        self.arm_pub.publish(self.m5_angle)
        self.speak('I start storing groceries.')
        while self.front_laser_dist > 0.5 and not rospy.is_shutdown():
            print 'Advancing!'
            self.twist_cmd.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.1)
        self.twist_cmd.linear.x = 0
        rospy.sleep(2.0)
        place = String()
        place.data = 'cupboard'
        self.navigation_memorize_pub.publish(place)
        print 'Memorizing!'
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.navigation_result_flg = False
        rospy.sleep(1.0)
        self.speak('I reached the cup board.')
        self.speak('Can you open the cup board door?')
        min_laser = self.min_laser_dist
        while min_laser - 0.2 < self.min_laser_dist and not rospy.is_shutdown():
            print 'Wait For The Cupboard Door To Open!  :  ', self.min_laser_dist
            time.sleep(1.0)
        rospy.sleep(2.0)        
        self.speak('Thank you for your help.')
        for i in range(1):
            self.twist_cmd.linear.x = -0.2
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.5)
        self.twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(self.twist_cmd)
        for i in range (3):
            self.twist_cmd.angular.z = 2.0
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy .sleep(0.5)
        self.twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.twist_cmd)
        return 1
    
    def findTable(self):#---------------------------------------state 1
        print 'State : 1'
        self.m5_angle.data = 0.8
        self.arm_pub.publish(self.m5_angle)
        rospy.sleep(2.0)
        list_req = Bool()
        list_req.data = True
        self.twist_cmd.angular.z = 1.0
        self.object_list_req_pub.publish(list_req)
        print 'Object_num : ', self.object_num
        count = 0
        #object探索
        while self.object_num < 2 and not rospy.is_shutdown():
            count += 1
            print count
            self.object_list = []
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(2.0)
            self.object_list_req_pub.publish(list_req)
            rospy.sleep(1.0)
            print 'Object_num : ', self.object_num
            print 'Waiting For Object Recognize!'
            #詰み回避用
            if count >= 8: #ここの数値で回転する回数を調整
                count = 0
                self.m5_angle.data = 0.6
                self.arm_pub.publish(self.m5_angle)
                self.twist_cmd.angular.z *= -1
                rospy.sleep(3.0)            
        self.object_num = -1
        self.twist_cmd.angular.z = 0.0
        if self.m5_angle.data == 0.6:
            self.twist_cmd.linear.x = 0.2
            for i in range(4):
                self.cmd_vel_pub.publish(self.twist_cmd)
                rospy.sleep(0.5)
            self.twist_cmd.linear.x = 0
            return 1 #this state
        self.twist_cmd.linear.x = 0.2
        for i in range(2):
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.5)
        self.twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(self.twist_cmd)
        place = String()
        place.data = 'table'
        self.navigation_memorize_pub.publish(place)
        print 'Memorizing!'
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.navigation_result_flg = False
        self.speak('I found the table.')
        return 3

    def approachTable(self):#-----------------------------------state 2
        print 'State : 2'
        place = String()
        place.data = 'table'
        self.navigation_command_pub.publish(place)
        print 'Moving!'
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.navigation_result_flg = False
        list_req = Bool()
        list_req.data = True
        self.twist_cmd.angular.z = 1.0
        count = 3
        self.object_list_req_pub.publish(list_req)
        time.sleep(2.0)
        while self.object_num < 2 and not rospy.is_shutdown():
            count += 1
            print count
            self.object_list = []
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(2.0)
            self.object_list_req_pub.publish(list_req)
            rospy.sleep(1.0)
            print 'Object_num : ', self.object_num
            print 'Looking For Objects!'
            if count >= 6:
                count = 0
                self.twist_cmd.angular.z *= -1
        self.speak('I found objects.')
        self.object_num = -1
        self.twist_cmd.linear.x = 0
        self.twist_cmd.angular.z = 0
        return 3

    def graspObject(self):#-------------------------------------state 3
        print 'State : 3'
        object_name = self.object_list[0]
        self.speak('I grasp the ' + object_name)
        grasp_req = String()
        grasp_req.data = object_name
        self.object_grasp_req_pub.publish(grasp_req)
        print 'Wait Object Recognition!'
        while self.object_grasp_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.object_grasp_result_flg = False
        self.object_list = []
        pose_req = String()
        pose_req.data = 'carry'
        self.changing_pose_pub.publish(pose_req)
        rospy.sleep(1.0)
        return 4
    
    def placeObject(self):#-----------------------------------state 4
        print 'state : 4'
        place = String()
        place.data = 'cupboard'
        self.navigation_command_pub.publish(place)
        print self.navigation_result_flg
        print 'Moving!'
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.navigation_result_flg = False
        #placingは完全決め打ちにします
        '''
        list_req = Bool()
        list_req.data = True
        self.twist_cmd.linear.x = 0
        self.twist_cmd.angular.z = -1.0
        count = 2
        self.object_list_req_pub.publish(list_req)
        time.sleep(3.0)
        while self.object_num < 1 and not rospy.is_shutdown():
            count += 1
            print count
            self.object_list = []
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(2.0)
            self.object_list_req_pub.publish(list_req)
            rospy.sleep(1.0)
            print 'Object_num : ', self.object_num
            print 'Cupboard searching!'
            if count >= 4:
                count = 0
                self.twist_cmd.angular.z *= -1
        self.object_list = []
        self.object_num = -1
        '''
        self.twist_cmd.linear.x = -0.2
        self.twist_cmd.angular.z = 0
        while self.front_laser_dist < 0.6 and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_cmd)
            time.sleep(0.1)
        place_req = Bool()
        place_req.data = True
        self.speak('I place object.')
        self.object_place_req_pub.publish(place_req)
        while self.object_place_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        rospy.sleep(3.0)
        self.object_place_flg = False
        self.twist_cmd.linear.x = 0.1
        self.twist_cmd.angular.z = 0
        while self.front_laser_dist > 0.35 and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.1)
        rospy.sleep(1.0)
        self.m4_angle.data = 0.0
        self.arm_pub.publish(self.m4_angle)
        time.sleep(1.0)
        self.twist_cmd.linear.x = -0.2
        self.twist_cmd.angular.z = 0
        while self.front_laser_dist < 0.6 and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.1)
        self.twist_cmd.linear.x = 0
        pose_req = String()
        pose_req.data = 'carry'
        self.changing_pose_pub.publish(pose_req)
        rospy.sleep(2.0)
        return 2

if __name__ == '__main__':
    rospy.init_node('Storing_Groceries')
    sg = StoringGroceries()
    state = 0
    while not rospy.is_shutdown():
        if state == 0:
            state = sg.inspectCupboard()
            #棚の扉を開けてもらう
        elif state == 1:
            state = sg.findTable()
            #テーブルを探す
        elif state == 2:
            state = sg.approachTable()
            #テーブルに移動
        elif state == 3:
            state = sg.graspObject()
            #オブジェクトをつかむ
        elif state == 4:
            state = sg.placeObject()
            #オブジェクトを置く
    #stateの遷移 : 0 ➤ 1 ➤ 3 ➤ 4 ➤ 2 ➤ 3 ➤ 4 ➤ 2 ➤ … と永遠に続けます
    #0,1は初回のみ 2は2週目から
