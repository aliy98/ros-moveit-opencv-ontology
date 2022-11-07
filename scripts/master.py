#!/usr/bin/env python
"""
.. module:: master
    :platform: Unix
    :synopsis: the main python script in ros_gmapping_movebase package

.. moduleauthor:: Ali Yousefi <aliyousef98@outlook.com>

Uses rosparam:
    /robot_state

This is the main python script in ros_gmapping_movebase package, it gets user request to choose robot
behaviour using robot_state rosparam. Everytime you choose a behaviour for robot you should continue
the process in the corresponding node shell, and then if you want to change the robot behaviour again
you must come back to this shell.

"""

import rospy
from std_srvs.srv import *
import os

# 1 - movebase client
# 2 - teleop keyboard
# 3 - assisted teleop

def change_state():
    """
    Function for changing robot's behaviour and sending the corresponding response to other three nodes

    robot_state can have valuse from 0 to 3 with the following rules:

            0: waiting for user to choose robot behaviour

            1: movebase client 

            2: teleop twist keyboard without obstacle avoidance

            3: teleop twist keyboard with obstacle avoidance

    """
    os.system('cls||clear')
    print("** MASTER NODE **\n")
    # gets robot behaviour from user
    x = input('''Choose robot behaviour:
    1. reach point(x,y) autonomously
    2. drive with keyboard
    3. drive robot with collision avoidance\n
    input: ''')

    if x == '1':
        rospy.set_param('robot_state', '1')
        print("\nstate changed: movebase client")
    elif x == '2':
        rospy.set_param('robot_state', '2')
        print("\nstate changed: teleop keyboard")
    elif x == '3':
        rospy.set_param('robot_state', '3')
        print("\nstate changed: assisted teleop")


def main():
    rospy.init_node('master')
    rospy.set_param('robot_state', '0')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if rospy.get_param('robot_state')=='0':
            change_state()
        else:
            rate.sleep()
            continue
        rate.sleep()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    main()