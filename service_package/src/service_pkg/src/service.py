#!/usr/bin/env python

import rospy
import copy

import actionlib
from std_msgs.msg import Int32, String
#from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from actionlib_msgs.msg import GoalStatusArray

class ServiceController(object):
    def __init__(self):

        # poses for each goals: pos x y and quaternion z w
        self.home_pose = [0.0, 0.0, 0.0, 1.0]

        table1_pose = [-1.8, -1.0, 0.0, 1.0]
        table2_pose = [-0.3, -2.2, 1.0, 0.0]
        table3_pose = [-1.8, -3.2, 0.0, 1,0]
        self.pose_dict = {'home': self.home_pose, 'table1': table1_pose, 'table2': table2_pose, 'table3': table3_pose}

        # variables initialisation
        self.init_menu = "standby"
        self.init_destination = "home"

        self.weight = 0.0
        self.weight_threshold = 50
        self.menu_weights = {'menu1': 350, 'menu2': 650, 'menu3': 1100, 'return': 200, 'standby': 5000}
        self.menu = copy.deepcopy(self.init_menu)
        self.destination = copy.deepcopy(self.init_destination)
        self.service_state = "goto_home"

        # action client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # subscriber
        # from GUI: which table to go to?
        self.destination_sub = rospy.Subscriber('/gui_table', String, self.destination_callback)
        # from GUI: which menu to be served?
        self.menu_weight_sub = rospy.Subscriber('/menu', String, self.menu_callback)
        # from loadcell: what's the weight?
        self.weight_sub = rospy.Subscriber('/tb3/weight', Int32, self.weight_callback)

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

        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        print("Goal sent!")
        wait = self.ac.wait_for_result()
        if not wait:
            print("Action server unavailable")

    # subscriber callback functions
    def destination_callback(self, data):
        self.destination = data.data

    def menu_callback(self, data):
        self.menu = data.data

    def weight_callback(self, data):
        self.weight = data.data


def main():
    # node initialisation
    rospy.init_node("service_controller", anonymous=False)
    # set loop rate
    rate = rospy.Rate(20)
    # class instantiation
    sc = ServiceController()

    while not rospy.is_shutdown():
        if sc.service_state == "goto_home":
            # TODO: initialise autopark when home pose is reached
            # TODO: standby_home when autopark is done
            print("---")
            print("Sending goal to home")
            sc.move_to(sc.home_pose)

            print("Goal reached! Entering standby...")
            sc.service_state = "standby_home"

            sc.menu = copy.deepcopy(sc.init_menu)
            sc.destination = copy.deepcopy(sc.init_destination)

        elif sc.service_state == "standby_home":
            # read for gui_menu and weight
            # if weight > sc.menu_weights[menu] then publish goto depending on gui_table
            if sc.menu == "return" and sc.weight < sc.menu_weights['return']:
                print("---")
                print("Return request received - waiting 5 sec")
                rospy.sleep(5)
                print("Initialising navigation to " + sc.destination)
                sc.service_state = "goto_table"
            elif sc.menu != "return" and sc.weight > sc.menu_weights[sc.menu]:
                print("---")
                rospy.sleep(3)
                print("Load detected: " + str(sc.weight))
                print("Loaded: " + sc.menu)
                print("Initialising navigation to " + sc.destination)
                sc.service_state = "goto_table"

        elif sc.service_state == "goto_table":
            print("---")
            print("Sending goal to " + sc.destination)
            sc.move_to(sc.pose_dict[sc.destination])

            print("Goal reached! Entering standby...")
            sc.service_state = "standby_table"

        elif sc.service_state == "standby_table":
            # read for weight
            # if weight < threshold then goto_home
            if sc.menu == "return" and sc.weight > sc.menu_weights["return"]:
                print("---")
                rospy.sleep(2)
                print("Load detected: " + str(sc.weight))
                rospy.sleep(3)
                print("Initialising navigation to home")
                sc.service_state = "goto_home"
            if sc.menu != "return" and sc.weight < sc.weight_threshold:
                print("---")
                rospy.sleep(3)
                print("Load detected: " + str(sc.weight))
                print("Initialising navigation to home")
                sc.service_state = "goto_home"
                
        rate.sleep()

if __name__ == '__main__':
    main()