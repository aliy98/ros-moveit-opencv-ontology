#!/usr/bin/env python
from os.path import dirname, realpath
import rospy
import smach
import smach_ros
import time
import ontological_robot_control
from ontological_robot_control.msg import PlanGoal
from ontological_robot_control.srv import SetBatteryLevel
from ontological_robot_control import architecture_name_mapper as anm
from ontological_robot_control.topological_map import TopologicalMap

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 2

def move_to_room(room):
    log_msg = 'Received request for robot to move to ' + room
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    if room   == 'E':
        move_to_pose(3.5, 1.0)
    elif room == 'R1':
        move_to_pose(1.0, 4.5)
    elif room == 'R2': 
        move_to_pose(1.0, 7.5)
    elif room == 'R3':
        move_to_pose(9.0, 4.5) 
    elif room == 'R4':
        move_to_pose(9.0, 7.5) 
    elif room == 'C1':
        move_to_pose(3.0, 4.5) 
    elif room == 'C2':
        move_to_pose(6.5, 4.5)

def move_to_pose(x, y):
    goal = PlanGoal()
    goal.target.x = x
    goal.target.y = y
    pub.publish(goal)
    log_msg = 'Request sent to planner server'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

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

# define state Room E
class RoomE(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D6','D7', 'stay'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        _set_battery_level_client(20)
        log_msg = f'Battery Charged.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room == "C1" or target_room == "R1" or target_room == "R2":
            move_to_room('C1')
            return 'D7'
        elif target_room == "C2" or target_room == "R3" or target_room == "R4":
            move_to_room('C2')
            return 'D6'
        else:
            return 'stay'

# define state Room 1
class Room1(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D1', 'stay'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R1":
            move_to_room('C1')
            return 'D1'
        else:
            return 'stay'

# define state Room 2
class Room2(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D2', 'stay'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R2":
            move_to_room('C1')
            return 'D2'
        else:
            return 'stay'

# define state Room 3
class Room3(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D3', 'stay'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R3":
            move_to_room('C2')
            return 'D3'
        else:
            return 'stay'

# define state Room 4
class Room4(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D4', 'stay'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R4":
            move_to_room('C2')
            return 'D4'
        else:
            return 'stay'
    
# define state Corridor 1
class Corridor1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['D1','D2','D5','D7','stay'])

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room == "R1":
            move_to_room('R1')
            return 'D1'
        elif target_room == "R2":
            move_to_room('R2')
            return 'D2'
        elif target_room == "C2" or target_room == "R3" or target_room == "R4":
            move_to_room('C2')
            return 'D5'
        elif target_room == "E":
            move_to_room('E')
            return 'D7'
        else:
            return 'stay'

# define state Corridor 2
class Corridor2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['D3','D4','D5','D6','stay'])

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room == "R3":
            move_to_room('R3')
            return 'D3'
        elif target_room == "R4":
            move_to_room('R4')
            return 'D4'
        elif target_room == "C1" or target_room == "R1" or target_room == "R2":
            move_to_room('C1')
            return 'D5'
        elif target_room == "E":
            move_to_room('E')
            return 'D6'
        else:
            return 'stay'
        
if __name__ == '__main__':
    # Initialise this node.
    rospy.init_node(anm.NODE_FINITE_STATE_MACHINE, log_level=rospy.INFO)
    now = rospy.get_rostime()
    tm = TopologicalMap(LOG_TAG, now)

    # Publish target point to planner
    pub = rospy.Publisher('/target_point', ontological_robot_control.msg.PlanGoal, queue_size=10)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('E', RoomE(), transitions={'D7':'C1', 'D6':'C2', 'stay':'E'})
        smach.StateMachine.add('C1', Corridor1(), transitions={'D7':'E', 'D1':'R1', 'D2':'R2', 'D5':'C2', 'stay':'C1'})
        smach.StateMachine.add('C2', Corridor2(), transitions={'D6':'E', 'D3':'R3', 'D4':'R4', 'D5':'C1', 'stay':'C2'})
        smach.StateMachine.add('R1', Room1(), transitions={'D1':'C1', 'stay':'R1'})
        smach.StateMachine.add('R2', Room2(), transitions={'D2':'C1', 'stay':'R2'})
        smach.StateMachine.add('R3', Room3(), transitions={'D3':'C2', 'stay':'R3'})
        smach.StateMachine.add('R4', Room4(), transitions={'D4':'C2', 'stay':'R4'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()