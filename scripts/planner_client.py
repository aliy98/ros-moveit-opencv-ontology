#!/usr/bin/env python

import rospy
import time
# Import the ActionServer implementation used.
from actionlib import SimpleActionClient
# Import the messages used by services and publishers.
from ontological_robot_control.msg import PlanGoal
# Import constant name defined to structure the architecture.
from ontological_robot_control import architecture_name_mapper as anm
import ontological_robot_control  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionClient`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER_CLIENT

def planner_client_callback(data):
    plan_result = planner_client(data.target.x, data.target.y)
    # log_msg = 'Plan Result:'
    # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # print(plan_result)

    pub.publish(plan_result)

# It uses the planner action server and cancels it if necessary.
def planner_client(x, y):
    # Create an action client called "planner_client" with action definition file "arch_skeleton.msg.PlanAction"
    client = SimpleActionClient(anm.ACTION_PLANNER,ontological_robot_control.msg.PlanAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    log_msg = 'waiting for planner server'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    goal = PlanGoal()
    goal.target.x = x
    goal.target.y = y
    # Sends the goal to the action server.
    client.send_goal(goal)
    log_msg = 'waiting for planner to find the path'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))
    # detects if the plan is found before timeout
    if finished_before_timeout:
        log_msg = 'Plan found!'
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
    rospy.init_node(anm.NODE_PLANNER_CLIENT, log_level=rospy.INFO)
    rospy.Subscriber('/target_point', ontological_robot_control.msg.PlanGoal, planner_client_callback)
    pub = rospy.Publisher('/path', ontological_robot_control.msg.PlanResult, queue_size=10)
    rospy.spin()