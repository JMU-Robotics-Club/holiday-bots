#!/usr/bin/env python
"""Demo of sending navigation goals using actionlib. 

Holiday Mapping Project

Usage: 

action_nav.py TARGET_X TARGET_Y [THETA]

Author: Nathan Sprague & Friends
Version: Holiday Version

"""
import sys
import rospy
import actionlib
import tf

from sound_play.libsoundplay import SoundClient

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus

class NavNode(object):

    def __init__(self):
        """ Set up the node. """

        rospy.init_node('halloween_node')

        soundhandle = SoundClient()

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.state_names = {}
        self.state_names[GoalStatus.PENDING] = "PENDING"
        self.state_names[GoalStatus.ACTIVE] = "ACTIVE"
        self.state_names[GoalStatus.PREEMPTED] = "PREEMPTED"
        self.state_names[GoalStatus.SUCCEEDED] = "SUCCEEDED"
        self.state_names[GoalStatus.ABORTED] = "ABORTED"
        self.state_names[GoalStatus.REJECTED] = "REJECTED"
        self.state_names[GoalStatus.RECALLED] = "RECALLED"
        self.state_names[GoalStatus.LOST] = "LOST"


    def goal_message(self, x_target, y_target, theta_target):
        """ Create a goal message in the base_link coordinate frame"""

        quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
        # Create a goal message ...
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = x_target
        goal.target_pose.pose.position.y = y_target
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal

    def goto_point(self, x_target, y_target, theta_target=0):
        """ Move to a location relative to the robot's current position """

        rospy.loginfo("navigating to: ({},{})".format(x_target, y_target))

        goal = self.goal_message(x_target, y_target, theta_target)

        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()

        rospy.loginfo("Sending goal.")
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")

        # Check in after a while to see how things are going.
        rospy.sleep(1.0)
        rospy.loginfo("Status Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "ACTIVE"
        state_name = self.state_names[self.ac.get_state()]
        rospy.loginfo("State      : {}".format(state_name))

        # Wait until the server reports a result.
        self.ac.wait_for_result()
        rospy.loginfo("Result Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "SUCCEEDED" or "ABORTED"
        state_name = self.state_names[self.ac.get_state()]
        rospy.loginfo("State      : {}".format(state_name))



if __name__ == "__main__":
    p1 = (1,2,40, 'Bernstein')
    p2 = (1,2,40, 'Bowers')
    p3 = (1,2,40, 'Sprague')
    p4 = (1,2,40, 'Aboutabl')
    p5 = (1,2,40, 'Fox')
    p6 = (1,2,40, 'Simmons')

    points = []
    points.append(p1)
    points.append(p2)
    points.append(p3)
    points.append(p4)
    points.append(p5)
    points.append(p6)

    nav_node = NavNode()

    for x in points:
        nav_node.goto_point()
""" Loop needs a wait delay, and it needs a sound output using the speech stuff"""
    if len(sys.argv) == 3:
        nav_node.goto_point(float(sys.argv[1]), float(sys.argv[2]))
    else:
        nav_node.goto_point(float(sys.argv[1]), float(sys.argv[2]),
                            float(sys.argv[3]))
