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

class StoringGroceries:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place',String,queue_size=1)
        self.navigation_command_pub = rospy.Publisher('/navigation/command_place',String,queue_size=1)
        self.pdf_create_pub = rospy.Publisher('/pdf/create_req',Bool,queue_size=1)
        self.pdf_append_pub = rospy.Publisher('/pdf/append_req',Bool,queue_size=1)
        self.object_recog_req_pub = rospy.Publisher('/object/recog_req',String,queue_size=1)#searchの開始
        self.object_list_req_pub = rospy.Publisher('/object/list_req',Bool,queue_size=1)#objectのリストをもらう
        self.object_grasp_req_pub = rospy.Publisher('/object/grasp_req',String,queue_size=10)#manipulationの開始
        self.object_image_generate_req_pub = rospy.Publisher('/object/image_generate_req',Bool,queue_size=1)#objectの画像を保存
        self.object_count_req_pub = rospy.Publisher('/object/count_req',Bool,queue_size=1)#objectの個数を要求
#        self.changing_pose_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)#manipulateしたあとの変形
        self.object_place_req_pub = rospy.Publisher('/arm/place_req',Bool,queue_size=1)#objectを置く?

        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)        
        self.pdf_res_sub = rospy.Subscriber('/pdf_result',Bool,self.pdf_resultCB)
        self.object_recog_res_sub = rospy.Subscriber('/object/recog_res',Bool,self.ObjectRecogResultCB)
        self.object_list_res_sub = rospy.Subscriber('/object/list_res',String,self.ObjectListCB)
        self.grasp_res_sub = rospy.Subscriber('/object/grasp_res',Bool,self.ObjectGraspResultCB)
        self.object_image_generate_res_sub = rospy.Subscriber('/object/image_generate_res',Bool,self.ObjectImageGenerateResultCB)
        self.object_count_res_sub = rospy.Subscriber('/object/count_res',Int8,self.ObjectCountCB)
#        self.('/arm/change/result',String,self.)
        self.object_place_res_sub = rospy.Subscriber('/object/place_res',Bool,self.ObjectPlaceCB)
        self.navigation_res_sub = rospy.Subscriber('/navigation/result',Bool,self.NavigationResultCB)

#        self.min_laser_dist = 999.9
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

    def LaserCB(self,laser_scan):
#        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]

    def pdf_resultCB(self,result_msg):
        self.pdf_result_flg = True

    def ObjectRecogResultCB(self,result_msg):
        self.object_recog_flg = True

    def ObjectListCB(self,result_msg):
        self.object_list = result_msg.data.split(' ')
        print self.object_list
        self.object_list_flg = True
        
    def ObjectGraspResultCB(self,result_msg):
        self.object_grasp_result_flg = True

    def ObjectImageGenerateResultCB(self,result_msg):
        self.object_image_generate_result_flg = True

    def ObjectCountCB(self,result_msg):
        print 'in'
        self.object_num = result_msg.data

    def ObjectPlaceCB(self,result_msg):
        self.object_place_flg = True

    def NavigationResultCB(self,result):
        self.navigation_result_flg = True

    def InspectCupboard(self):#-----------------state 0
        print 'state 0'
        time.sleep(3.0)
        voice_cmd = '/usr/bin/picospeaker %s' % 'I start storing groceries.'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        voice_cmd = '/usr/bin/picospeaker %s' % 'Can you open the cup board door?'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        voice_cmd = '/usr/bin/picospeaker %s' % 'Thank you for your help.'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        twist_cmd = Twist()
        while self.front_laser_dist > 0.7 and not rospy.is_shutdown():
            print 'laser'
#            twist_cmd.linear.x = self.front_laser_dist - 0.3
            twist_cmd.linear.x = 0.2
            twist_cmd.angular.z = 0
            self.cmd_vel_pub.publish(twist_cmd)
            rospy.sleep(0.1)
        twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(twist_cmd)
        image_req = Bool()
        image_req.data = True
        print 'publish'
        self.object_image_generate_req_pub.publish(image_req)
        while self.object_image_generate_result_flg == False and not rospy.is_shutdown():
            print 'image generating'
            time.sleep(3.0)
        self.object_image_generate_result_flg = False
        pdf_req = Bool()
        pdf_req.data = True
        self.pdf_create_pub.publish(pdf_req)
        voice_cmd = '/usr/bin/picospeaker %s' % 'I reached the cup board.'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        voice_cmd = '/usr/bin/picospeaker %s' % 'I remember this location.'
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        place = String()
        place.data = 'cupboard'
        self.navigation_memorize_pub.publish(place)
        while (self.pdf_result_flg == False or self.navigation_result_flg == False) and not rospy.is_shutdown():
            print 'wait pdf create and memorize location.'
            print 'pdf:', self.pdf_result_flg, 'navi', self.navigation_result_flg
            time.sleep(3.0)
        self.pdf_result_flg = False
#        while self.navigation_result_flg == False and not rospy.is_shutdown():
#            print 'navi', self.navigation_result_flg
#            time.sleep(3.0)
        self.navigation_result_flg = False
        twist_cmd = Twist()
        twist_cmd.linear.x = -1.0
        self.cmd_vel_pub.publish(twist_cmd)
        time.sleep(6)
        twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(twist_cmd)
        twist_cmd.angular.z = 2.0
        self.cmd_vel_pub.publish(twist_cmd)
        time.sleep(8)
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
        return 1
    
    def FindTable(self):#-----------------------state 1
        print 'state1'
        count_req = Bool()
        count_req.data = True
        self.object_list_req_pub.publish(count_req)
        while self.object_num < 3 and not rospy.is_shutdown():
            print 'object_num', self.object_num
            twist_cmd = Twist()
            twist_cmd.angular.z = 1.0
            self.cmd_vel_pub.publish(twist_cmd)
            time.sleep(3)
            twist_cmd.angular.z = 0
            self.cmd_vel_pub.publish(twist_cmd)
            self.object_clist_req_pub.publish(count_req)
            time.sleep(1)
            print 'waiting for object_recognizer'
        self.object_num = -1
        place = String()
        place.data = 'table'
        self.navigation_memorize_pub.publish(place)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            print 'memorizing'
            time.sleep(3.0)
        self.navigation_result_flg = False
        voice_cmd = '/usr/bin/picospeaker %s' % 'I found the table.'
        subprocess.call(voice_cmd.strip().split(' '))
        return 3

        '''
        print 'state 1'
        twist_cmd = Twist()
        twist_cmd.angular.z = 2.0
        self.cmd_vel_pub.publish(twist_cmd)
        time.sleep(3)
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
        count_req = Bool()
        count_req.data = True
        self.object_count_req_pub.publish(count_req)
        while self.object_num == -1 and not rospy.is_shutdown():
            print 'waiting for object_recognizer'
            time.sleep(1)
        self.object_recog_flg = False
        if self.object_num > 3:
            voice_cmd = '/usr/bin/picospeaker %s' % 'I found the table.'
#            subprocess.call(voice_cmd.strip().split(' '))
            place = String()
            place.data = 'table'
            self.navigation_memorize_pub.publish(place)
            while self.navigation_result_flg == False and not rospy.is_shutdown():
                print 'memorizing'
                time.sleep(3.0)
            self.navigation_result_flg = False
            self.object_num = -1
            return 3
        voice_cmd = '/usr/bin/picospeaker %s' % "I couldn't find the table."
#        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        self.object_num = -1
        return 2
        '''
    
    def ApproachTable(self):#-------------------state 2
        print 'state 2'
        place = String()
        place.data = 'table'
        self.navigation_command_pub.publish(place)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            print 'moving'
            time.sleep(3.0)
        self.navigation_result_flg = False
        count_req = Bool()
        count_req.data = True
        while self.object_num < 3 and not rospy.is_shutdown():
            twist_cmd = Twist()
            twist_cmd.angular.z = 1.0
            self.cmd_vel_pub.publish(twist_cmd)
            time.sleep(3)
            twist_cmd.angular.z = 0
            self.cmd_vel_pub.publish(twist_cmd)
            self.object_count_req_pub.publish(count_req)
            time.sleep(1)
            print 'looking for objects'
        voice_cmd = '/usr/bin/picospeaker %s' % 'I found objects.'
        subprocess.call(voice_cmd.strip().split(' '))
        self.object_num = -1
        return 3

        '''
        recog_req = String()
        recog_req.data = ''
        self.object_recog_req_pub.publish(recog_req)
        while self.object_recog_flg == False and not rospy.is_shutdown():
            print 'waiting for object_recognizer'
            time.sleep(1)
        self.object_recog_flg = False
        count_req = Bool()
        count_req.data = True
        self.object_count_req_pub.publish(count_req)
        while self.object_num == -1 and not rospy.is_shutdown():
            print 'counting the number of objects'
            time.sleep(1)
        if self.object_num > 3:
            self.object_num = -1
            return 3
        self.object_num = -1
        twist_cmd = Twist()
        twist_cmd.angular.z = 2.0
        self.cmd_vel_pub.publish(twist_cmd)
        time.sleep(3)
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
        return 1
        '''
    
    def GraspObject(self):#---------------------state 3
        print 'state 3'
#        with open('/~~~/catkin_ws/darknet/object_point.txt') as f:
#            obj = f.readlines()
#        f.close()
#        a, b, c, d, object_neme = obj[0].split()
        list_req = Bool()
        list_req.data = True
        self.object_list_req_pub.publish(list_req)
        while len(self.object_list) <= 3 and not rospy.is_shutdown():
            print 'getting object list'
            time.sleep(3.0)
        self.object_list_flg = False
        object_name = self.object_list[0]
        voice_cmd = '/usr/bin/picospeaker %s' % 'I grasp the ' + object_name
        subprocess.call(voice_cmd.strip().split(' '))
        time.sleep(3.0)
        grasp_req = String()
        grasp_req = object_name
        self.object_grasp_req_pub.publish(grasp_req)
        while self.object_grasp_result_flg == False and not rospy.is_shutdown():
            print 'wait Object Recognition'
            time.sleep(3.0)
        self.object_grasp_result_flg = False
        self.object_list = []
        return 4
    
    def ReleaseObject(self):#-------------------state 4
        print 'state 4'
        place = String()
        place.data = 'cupboard'
        self.navigation_command_pub.publish(place)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            print 'moving'
            time.sleep(3.0)
        self.navigation_result_flg = False
        place_req = Bool()
        place_req = True
        self.object_place_req_pub.publish(place_req)
        while self.object_place_flg == False and not rospy.is_shutdown():
            print 'object placing'
            time.sleep(3.0)
        self.object_place_flg = False
        image_req = Bool()
        image_req.data = True
        self.object_image_generate_req_pub.publish(image_req)
        while self.object_image_generate_result_flg == False and not rospy.is_shutdown():
            print 'image generating'
            time.sleep(3.0)
        self.object_image_generate_result_flg = False
        pdf_req = Bool()
        pdf_req.data = True
        self.pdf_append_pub.publish(pdf_req)
        while self.pdf_result_flg == False and not rospy.is_shutdown():
            print 'pdf appending'
            time.sleep(3.0)
        self.pdf_resutl_flg = False
#        return 999
        return 2

if __name__ == '__main__':
    rospy.init_node('Storing_Groceries')
    sg = StoringGroceries()
    state = 0
    while not rospy.is_shutdown():
#        while not state == 999 and not rospy.is_shutdown():
        if state == 0:
            state = sg.InspectCupboard()
            #棚の扉を開けてもらう
        elif state == 1:
            state = sg.FindTable()
            #テーブルを探す
        elif state == 2:
            state = sg.ApproachTable()
            #テーブルに移動
        elif state == 3:
            state = sg.GraspObject()
            #オブジェクトをつかむ
        elif state == 4:
            state = sg.ReleaseObject()
            #オブジェクトを置く
#            state = 2
#        rospy.sleep()
