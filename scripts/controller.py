#! /usr/bin/env python

import rospy
# Import constant name defined to structure the architecture.
from ontological_robot_control import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from ontological_robot_control.msg import Point, ControlFeedback, ControlResult
from ontological_robot_control.srv import GetPose, SetPose, SetBatteryLevel, GetBatteryLevel
import ontological_robot_control  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
from math import sqrt

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER

# An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `robot-state` node.
class ControllingAction(object):

    def __init__(self):
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      ontological_robot_control.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
                   f'between each via point spanning.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in 
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for target_point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = target_point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            robot_speed = 5
            pose = Point()
            pose = _get_pose_client()
            euc_dist = sqrt((target_point.x - pose.x)**2 + (target_point.y - pose.y)**2)
            delay = euc_dist / robot_speed
            rospy.sleep(delay)
            _set_pose_client(target_point)
            _consume_battery()
            # Log current robot position.
            log_msg = f'Reaching point ({target_point.x}, {target_point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.


# Update the current robot `pose` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
def _set_pose_client(pose):
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Log service call.
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

# Update the current robot `battery level` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
def _set_battery_level_client(battery_level):
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_BATTERY_LEVEL)
    try:
        # Log service call.
        log_msg = f'Set current robot battery level to the `{anm.SERVER_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(battery_level)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

# Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
def _get_pose_client():
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        # Log service response.
        log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({pose.x}, {pose.y}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return pose
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

# Retrieve the current robot battery level by the `state/battery_level` server of the `robot-state` node.
def _get_battery_level_client():
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_BATTERY_LEVEL)
    try:
        # Call the service and get a response with the current robot battery level.
        service = rospy.ServiceProxy(anm.SERVER_GET_BATTERY_LEVEL, GetBatteryLevel)
        response = service()
        battery_level = response.battery_level
        # Log service response.
        log_msg = f'Retrieving current robot battery level from the `{anm.NODE_ROBOT_STATE}` node as: {battery_level}.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return battery_level
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def _consume_battery():
    battery_level = _get_battery_level_client()
    battery_level = battery_level - 1 
    _set_battery_level_client(battery_level)

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
