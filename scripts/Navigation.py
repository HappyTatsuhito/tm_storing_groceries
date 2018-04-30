#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import tf
import math
import actionlib
import std_srvs.srv
import time
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist,Quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

class Navigation:
    def __init__(self):
#        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.ScanCB)
        self.pose_sub = rospy.Subscriber('/tf',TFMessage,self.BaseCB)
        self.location_word_sub = rospy.Subscriber('/navigation/memorize_place',String,self.ReceiveLocation)
        self.request_sub = rospy.Subscriber('/navigation/command_place',String,self.NavigateToDestination)
        
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=10)
        self.navigation_result_pub = rospy.Publisher('/navigation/result',Bool,queue_size=1)
        self.reset_pub = rospy.Publisher('reconfiguration/input',Bool,queue_size=10)

        self.clear_costmap = rospy.ServiceProxy('move_base/clear_costmaps',std_srvs.srv.Empty)

        self.location_list = []
        self.robot_pose_x = 999
        self.robot_pose_y = 999
        self.cmd_vel = Twist()
        self.cmd_vel.angular.z = 0
        self.location_word = 'Null'
        self.location_pose_x = 0
        self.location_pose_y = 0
        self.location_append_num = 0

#    def ScanCB(self,receive_msg):

    def BaseCB(self,receive_msg):
        pose = receive_msg
        if pose.transforms[0].header.frame_id == 'odom':
            self.robot_pose_x = pose.transforms[0].transform.translation.x
            self.robot_pose_y = pose.transforms[0].transform.translation.y
            if self.location_word != 'Null':
                self.location_pose = self.robot_pose_x
                self.location_pose = self.robot_pose_y
                self.location_list.append([self.location_word,self.location_pose_x,self.location_pose_y,0])
            self.location_append_num += 1
            self.location_word = 'Null'
        navigate_result = Bool()
        navigate_result.data = True
        self.navigation_result_pub.publish(navigate_result)

    def ReceiveLocation(self,receive_msg):
        self.location_word = receive_msg.data
        print self.location_word

    def NavigateToDestination(self,receive_msg):
        location_num = -1
        for location_num_i in range(len(self.location_list)):
            print location_num_i#test
            if self.location_list[location_num_i][0] in destination.data:
                location_num = location_num_i
        if location_num == -1:
            print "not exist such object"
            return
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if ac.wait_for_server(rospy.Duration(5)) == 1:
            print "wait for action client rising up 0"
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'          # 地図座標系
        goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
        goal.target_pose.pose.position.x =  self.location_list[location_num][1]
        goal.target_pose.pose.position.y =  self.location_list[location_num][2]
        q = tf.transformations.quaternion_from_euler(0, 0, self.location_list[location_num][3])
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        ac.send_goal(goal);
        while not rospy.is_shutdown():
            print 'goal state is',ac.get_state()
            if ac.get_state() == 1 and reset_flg == 1:
                print 'Got out of the obstacle'
                rosoy.sleep(1)
            if ac.get_state() == 3:
                print "goal"
                rospy.sleep(3)
                result = Bool()
                result.data = True
                self.result_pub.publish(result)
                time.sleep(3)
                break                        
            if ac.get_state() == 4:
                print 'Buried in obstacles'
                self.reset_pub.publish(1)
                ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                if ac.wait_for_server(rospy.Duration(5)) == 1:
                    print "wait for action client rising up 1"
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'          # 地図座標系                                                                      

                goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻

                goal.target_pose.pose.position.x =  self.location_list[location_num][1]
                goal.target_pose.pose.position.y =  self.location_list[location_num][2]
                q = tf.transformations.quaternion_from_euler(0, 0, self.location_list[location_num][3])
                goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
                ac.send_goal(goal);
                      
            pro_dist_to_goal = dist_to_goal
        print "finish"
        navigate_result = Bool()
        navigate_result.data = True
        self.navigation_result_pub.publish(navigate_result)

        
if __name__ == '__main__':
    rospy.init_node('sg_navigation',anonymous=True)
    navigation = Navigation()
    rospy.spin()
