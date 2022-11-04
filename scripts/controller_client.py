#!/usr/bin/env python

import rospy
import time
# Import the ActionServer implementation used.
from actionlib import SimpleActionClient
# Import constant name defined to structure the architecture.
from ontological_robot_control import architecture_name_mapper as anm
import ontological_robot_control.msg  # This is required to pass the `ControlAction` type for instantiating the `SimpleActionClient`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER_CLIENT

def controller_client_callback(data):
    control_result =  controller_client(data)
    # log_msg = 'Control Result:'
    # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # print(control_result)

# It uses the controller action server and cancels it if necessary.
def controller_client(goal):
    # Create an action client called "controller_client" with action definition file "arch_skeleton.msg.ControlAction"
    client = SimpleActionClient(anm.ACTION_CONTROLLER,ontological_robot_control.msg.ControlAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    log_msg = 'waiting for controller server'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # Sends the goal to the action server.
    client.send_goal(goal)
    log_msg = 'waiting for robot to reach the target'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))
    # detects if the target is reached before timeout
    if finished_before_timeout:
        log_msg = 'Target Reached!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # time.sleep(3)
        return client.get_result()
    else:
        log_msg = 'Action did not finish before time out!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # time.sleep(3)
        client.cancel_all_goals()

if __name__ == '__main__':
    # Initialise this node.
    rospy.init_node(anm.NODE_CONTROLLER_CLIENT, log_level=rospy.INFO)
    rospy.Subscriber('/path', ontological_robot_control.msg.PlanResult, controller_client_callback)
    rospy.spin()