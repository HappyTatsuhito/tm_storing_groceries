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
from std_msgs.msg import String, Bool, Int8, Float64

class StoringGroceries:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place',String,queue_size=1)
        self.navigation_command_pub = rospy.Publisher('/navigation/command_place',String,queue_size=1)
        self.pdf_create_pub = rospy.Publisher('/pdf/create_req',Bool,queue_size=1)
        self.pdf_append_first_pub = rospy.Publisher('/pdf/append/first_req',Bool,queue_size=1)
        self.pdf_append_second_pub = rospy.Publisher('/pdf/append/second_req',Bool,queue_size=1)
        self.object_recog_req_pub = rospy.Publisher('/object/recog_req',String,queue_size=1)#searchの開始
        self.object_list_req_pub = rospy.Publisher('/object/list_req',Bool,queue_size=1)#objectのリストをもらう
        self.object_grasp_req_pub = rospy.Publisher('/object/grasp_req',String,queue_size=10)#manipulationの開始
        self.object_image_generate_req_pub = rospy.Publisher('/object/image_generate_req',Bool,queue_size=1)#objectの画像を保存
        self.object_count_req_pub = rospy.Publisher('/object/count_req',Bool,queue_size=1)#objectの個数を要求
        self.changing_pose_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)#manipulateしたあとの変形
        self.object_place_req_pub = rospy.Publisher('/object/place_req',Bool,queue_size=1)#objectを置く
        self.m5_pub = rospy.Publisher('/m5_controller/command',Float64,queue_size=1)
        self.m6_pub = rospy.Publisher('/m6_controller/command',Float64,queue_size=1)

        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)        
        self.pdf_res_sub = rospy.Subscriber('/pdf_result',Bool,self.pdf_resultCB)
        self.object_recog_res_sub = rospy.Subscriber('/object/recog_res',Bool,self.ObjectRecogResultCB)
        self.object_list_res_sub = rospy.Subscriber('/object/list_res',String,self.ObjectListCB)
        self.grasp_res_sub = rospy.Subscriber('/object/grasp_res',Bool,self.ObjectGraspResultCB)
        self.object_image_generate_res_sub = rospy.Subscriber('/object/image_generate_res',Bool,self.ObjectImageGenerateResultCB)
        self.object_count_res_sub = rospy.Subscriber('/object/count_res',Int8,self.ObjectCountCB)
        self.object_place_res_sub = rospy.Subscriber('/object/place_res',Bool,self.ObjectPlaceCB)
        self.navigation_res_sub = rospy.Subscriber('/navigation/result',Bool,self.NavigationResultCB)

        self.min_laser_dist = 999.9
        self.front_laser_dist = 999.9
        self.pdf_result_flg = False
        self.object_recog_flg = False
        self.object_list = []
        self.object_list_flg = False
        self.object_grasp_result_flg = False
        self.object_image_generate_result_flg = False
        self.object_num = -1
        self.object_place_flg = False
        self.navigation_result_flg = False
        self.m5_angle = Float64()
        self.m6_angle = Float64()
        self.twist_cmd = Twist()

    def getLaserCB(self,laser_scan):
        self.laser_dist = laser_scan.ranges
        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]

    def getPdfResultCB(self,result_msg):
        self.pdf_result_flg = True

    def getObjectRecogResultCB(self,result_msg):
        self.object_recog_flg = True

    def getObjectListCB(self,result_msg):
        self.object_list = result_msg.data.split(' ')
        self.object_list[-1:] = []
        print self.object_list
        self.object_num = len(self.object_list)
        self.object_list_flg = True
        
    def getObjectGraspResultCB(self,result_msg):
        self.object_grasp_result_flg = True

    def getObjectImageGenerateResultCB(self,result_msg):
        self.object_image_generate_result_flg = True

    def getObjectCountCB(self,result_msg):
        self.object_num = result_msg.data

    def getObjectPlaceCB(self,result_msg):
        self.object_place_flg = True

    def getNavigationResultCB(self,result):
        self.navigation_result_flg = True

    def inspectCupboard(self):#-----------------state 0
        print 'State 0'
        self.m6_angle.data = -0.8
        self.m6_pub.publish(self.m6_angle)
        voice_cmd = '/usr/bin/picospeaker %s' % 'I start storing groceries.'
        subprocess.call(voice_cmd.strip().split(' '))
        while self.front_laser_dist > 0.5 and not rospy.is_shutdown():
            print 'Advancing!'
            self.twist_cmd.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.1)
        self.twist_cmd.linear.x = 0
        place = String()
        place.data = 'cupboard'
        self.navigation_memorize_pub.publish(place)
        print 'Memorizing!'
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.navigation_result_flg = False
        rospy.sleep(1.0)
        while self.front_laser_dist < 0.80 and not rospy.is_shutdown():
            print 'Reverse'
            self.twist_cmd.linear.x = -0.2
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(0.1)
        self.twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(self.twist_cmd)
        time.sleep(3.0)
        voice_cmd = '/usr/bin/picospeaker %s' % 'I reached the cup board.'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        voice_cmd = '/usr/bin/picospeaker %s' % 'Can you open the cup board door?'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        min_laser_index = self.laser_dist.index(self.min_laser_dist)
        while min_laser_index == self.laser_dist.index(self.min_laser_dist) and not rospy.is_shutdown():
            print 'Wait For The Cupboard Door To Open!'
            time.sleep(1.0)
        rospy.sleep(2.0)        
        voice_cmd = '/usr/bin/picospeaker %s' % 'Thank you for your help.'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        self.m6_angle.data = 0.0
        self.m6_pub.publish(self.m6_angle)
        rospy.sleep(3.0)
        image_req = Bool()
        image_req.data = True
        self.object_image_generate_req_pub.publish(image_req)
        print 'Image Generating!'
        while self.object_image_generate_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.object_image_generate_result_flg = False
        pdf_req = Bool()
        pdf_req.data = True
        self.pdf_create_pub.publish(pdf_req)
        print 'PDF Create!'
        while self.pdf_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.pdf_result_flg = False
        for i in range(2):
            self.m6_angle.data = -0.4 * (i + 1)
            self.m6_pub.publish(self.m6_angle)
            rospy.sleep(3.0)
            self.object_image_generate_req_pub.publish(image_req)
            print 'Image Generating!'
            while self.object_image_generate_result_flg == False and not rospy.is_shutdown():
                time.sleep(0.5)
            self.object_image_generate_result_flg = False
            self.pdf_append_first_pub.publish(pdf_req)
            print 'PDF Append!'
            while self.pdf_result_flg == False and not rospy.is_shutdown():
                time.sleep(0.5)
            self.pdf_result_flg = False
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
    
    def findTable(self):#-----------------------state 1
        print 'State1'
        self.m6_angle.data = -0.8
        self.m6_pub.publish(self.m6_angle)
        rospy.sleep(2.0)
        list_req = Bool()
        list_req.data = True
        self.twist_cmd.angular.z = 1.0
        rospy.sleep(3.0)
        self.object_list_req_pub.publish(list_req)
        print 'Object_num : ', self.object_num
        count = 0
        while self.object_num < 3 and not rospy.is_shutdown():
            count += 1
            print count
            self.object_list = []
            self.cmd_vel_pub.publish(self.twist_cmd)
            rospy.sleep(2.0)
            self.object_list_req_pub.publish(list_req)
            rospy.sleep(1.0)
            print 'Object_num : ', self.object_num
            print 'Waiting For Object Recognize!'
            if count >= 15:
                count = 0
                self.m6_angle.data = -0.4
                self.twist_cmd.angular.z *= -1
                self.m6_pub.publish(self.m6_angle.data)
                rospy.sleep(3.0)            
        self.object_num = -1
        self.twist_cmd.angular.z = 0.0
        if self.m6_angle.data == -0.4:
            self.twist_cmd.linear.x = 0.2
            for i in range(4):
                self.cmd_vel_pub.publish(self.twist_cmd)
                rospy.sleep(0.5)
            self.twist_cmd.linear.x = 0
            return 1
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
        voice_cmd = '/usr/bin/picospeaker %s' % 'I found the table.'
        subprocess.call(voice_cmd.strip().split(' '))
        return 3

    def approachTable(self):#-------------------state 2
        print 'State 2'
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
        time.sleep(3.0)
        while self.object_num < 2 and not rospy.is_shutdown():
            count++
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
        voice_cmd = '/usr/bin/picospeaker %s' % 'I found objects.'
        subprocess.call(voice_cmd.strip().split(' '))
        self.object_num = -1
        self.twist_cmd.linear.x = 0
        self.twist_cmd.angular.z = 0
        return 3

    def graspObject(self):#---------------------state 3
        print 'State 3'
        object_name = self.object_list[0]
        voice_cmd = '/usr/bin/picospeaker %s' % 'I grasp the ' + object_name
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
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
    
    def releaseObject(self):#-------------------state 4
        print 'state 4'
        place = String()
        place.data = 'cupboard'
        self.navigation_command_pub.publish(place)
        print self.navigation_result_flg
        print 'Moving!'
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.navigation_result_flg = False
        list_req = Bool()
        list_req.data = True
        self.twist_cmd.linear.x = 0
        self.twist_cmd.angular.z = -1.0
        count = 2
        self.object_list_req_pub.publish(list_req)
        time.sleep(3.0)
        while self.object_num < 1 and not rospy.is_shutdown():
            conunt++
            print count
            self.object_list[]
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
        self.twist_cmd.linear.x = -0.2
        self.twist_cmd.angular.z = 0
        while self.front_laser_dist < 0.6 and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_cmd)
            time.sleep(0.1)
        place_req = Bool()
        place_req.data = True
        self.object_place_req_pub.publish(place_req)
        print 'Object Placing!'
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
        self.m5_angle.data = -0.4
        self.m5_pub.publish(self.m5_angle)
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
        image_req = Bool()
        image_req.data = True
        self.object_image_generate_req_pub.publish(image_req)
        print 'Image Generating!'
        while self.object_image_generate_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.object_image_generate_result_flg = False
        pdf_req = Bool()
        pdf_req.data = True
        self.pdf_append_second_pub.publish(pdf_req)
        print 'PDF Appending!'
        while self.pdf_result_flg == False and not rospy.is_shutdown():
            time.sleep(0.5)
        self.pdf_resutl_flg = False
        rospy.sleep(1.0)
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
            state = sg.releaseObject()
            #オブジェクトを置く
