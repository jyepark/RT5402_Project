#!/usr/bin/env python
# # -*- coding: utf8 -*-

import rospy
import actionlib
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import *

class ServiceController(object):
    def __init__(self):

        # poses for each goals: pos x y and quaternion z w
        self.home_pose = [0.0, 0.0, 0.0, 1.0]
        self.table1_pose = [-2.0, -1.0, 1.0, 0.0]
        self.table2_pose = [0.0, -2.0, 0.0, 1.0]
        self.table3_pose = [-2.0, -3.0, 1.0, 0,0]

        # variables initialisation
        self.weight = 0.0
        self.menu_weights = {'menu1': 600, 'menu2': 1200, 'menu3': 1700, 'return': 200, 'standby': 5000}
        self.menu = "standby"
        self.service_state = "goto_home"
        self.pose = self.home_pose
        self.move_seq = 100
        self.move_goal_reached = False

        # static variables
        self.epsilon = 0.005 # threshold condition for reaching destination
        self.weight_threshold = 100

        # action client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # subscriber
        # from GUI: which table to go to?
        self.gui_table_sub = rospy.Subscriber('/gui_table', String, self.gui_table_callback)
        # from GUI: which menu to be served?
        self.menu_weight_sub = rospy.Subscriber('/menu', String, self.menu_callback)
        # from loadcell: what's the weight?
        self.weight_sub = rospy.Subscriber('/weight', Float64, self.weight_callback)
        # from move_base: are we there yet?
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)

        # publisher
        # flag for autopark
        #self.parking_pub = rospy.Publisher('/parking', String, queue_size=10)
        pass

    # take goal pose and send action to move_base
    def move_to(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = goal_pose[0]
        goal.target_pose.pose.position.y = goal_pose[1]
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = goal_pose[2]
        goal.target_pose.pose.orientation.w = goal_pose[3]

        self.ac.send_goal(goal)

    # subscriber callback functions
    def gui_table_callback(self, data):
        self.gui_table = data.data

    def menu_callback(self, data):
        self.menu = data.data

    def weight_callback(self, data):
        self.weight = data.data

    def result_callback(self, data):
        # check if new message is published in actionlib
        if self.move_seq != data.header.seq and data.status.status == 3:
            self.move_seq = data.header.seq
            self.move_goal_reached = True


def main():
    # node initialisation
    rospy.init_node("service_controller", anonymous=True)
    # set loop rate
    rate = rospy.Rate(250)
    # class instantiation
    sc = ServiceController()

    publish_goal = True # so that goal pose is only published once

    while not rospy.is_shutdown():
        if sc.service_state == "goto_home":
            # publish home pose
            if publish_goal:
                sc.move_to(sc.home_pose)
                publish_goal = False
            # initialise autopark when home pose is reached
            # TODO
            # standby_home when autopark is done
            if sc.move_goal_reached:
                sc.move_goal_reached = False
                sc.service_state == "standby_home"
                # menu = standby
                sc.menu = "standby"
                # overwrite pose data
                # TODO
        if sc.service_state == "standby_home":
            # read for gui_menu and weight
            # if weight > sc.menu_weights[menu] then publish goto depending on gui_table
            if sc.weight > sc.menu_weights[sc.menu]:
                sc.service_state = "goto_" + sc.gui_table
                publish_goal = True
        if sc.service_state == "goto_table1":
            # navigate to table1
            if publish_goal:
                sc.move_to(sc.table1_pose)
                publish_goal = False
            # standby_table when pose is reached
            if sc.move_goal_reached:
                sc.move_goal_reached = False
                sc.service_state = "standby_table"
        if sc.service_state == "goto_table2":
            # navigate to table1
            if publish_goal:
                sc.move_to(sc.table2_pose)
                publish_goal = False
            # standby_table when pose is reached
            if sc.move_goal_reached:
                sc.move_goal_reached = False
                sc.service_state = "standby_table"
        if sc.service_state == "goto_table3":
            # navigate to table1
            if publish_goal:
                sc.move_to(sc.table3_pose)
                publish_goal = False
            # standby_table when pose is reached
            if sc.move_goal_reached:
                sc.move_goal_reached = False
                sc.service_state = "standby_table"
        if sc.service_state == "standby_table":
            # read for weight
            # if weight < threshold then goto_home
            return_flag =  sc.menu == "return" and sc.weight > sc.menu_weights["return"]
            serve_flag = sc.menu != "return" and sc.weight < sc.weight_threshold
            if return_flag or serve_flag:
                rospy.sleep(3)
                sc.service_state == "goto_home"
                publish_goal = True
                
        rate.sleep()

if __name__ == '__main__':
    main()