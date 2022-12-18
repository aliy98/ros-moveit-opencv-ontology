# ros-moveit-opencv-ontology
ROS package for patrolling an indoor environment with a mobile robot

## Introduction
This package is an experiment to use a topological map ontology for controling a robot using ROS. The ontology consists of an indoor environment
with multiple rooms and a mobile robot. You can get to know more about the detatils of the source code using the 
[documentation](https://aliy98.github.io/ros-moveit-opencv-ontology/) provided for this rospackage.

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/topological_map.jpg" width="300" title="topological map">
</p>

The robot starts in room E and by scanning the provided markers, it receives the information to build the semantic map, i.e., the name and center 
position of each room and the connections between them.

Once the semantic map is built, robot has to start moving among the rooms with the policy that each room that has not been visited for a long time,
would be selected as the target room. Everytime the robot gets to the target room, it has to scan the room environment as it did for the first time
with the markers.

When the robot battery is low, it goes to the charger which is placed in room E, sand wait for some times before to start again with the above behavior.

## Simulation Environment
The simulation software in this package is gazebo. The environment in which the robot should implement the 
scanning and patrolling behaviors, is shown in the following figures:

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/assignment_world.png" width="400" title="sim1">
</p>

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/assignment_world2.png" width="400" title="sim2">
</p>

## Software Architecture
The software architucture is represented in the following figure.

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/sofar.png" width="800" title="sofar">
</p>

The components of this software architucture can be described as follows:

### robot URDF
The robot URDF in this package consists of a mobile base link provided with two cylindrical wheels, caster front and a manipulator with 5dof.
This robot uses a laser scanner to detect the obstacles, and a camera for scanning the provided markers.

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/urdf.png" width="300" title="urdf">
</p>

### robot-state
In order to simulate the states of the robot and its stimulus, the approach presented in the [arch_skeleton](https://github.com/buoncubi/arch_skeleton)
example is used with some changes (e.g. battery level, base movement state).

### marker_publisher
The image processing part is done using [OpenCV](https://opencv.org/) with the method presented in 
[aruco_ros](https://github.com/CarmineD8/aruco_ros) In order to convert between ROS Image 
messages and OpenCV images, [cv_bridge](http://wiki.ros.org/cv_bridge) package is used.

### marker_server
Implemnts a service which responses the information for each room in the semantic map by getting
call request with argument indicationg the room id.

### move_group
The controller node for robot manipulator arm. The package ``my_moveit`` which is auto generated by 
[MoveIt](https://moveit.ros.org/) for robot URDF, implements this node. It provides joint trajectory effort 
controllers for robot arm in order to reach the desired joints configuration. 

### move_base 
The node which is responsible for navigation. In this package, [move_base](http://wiki.ros.org/move_base) node is
used for finding the path between robot current position and target room position. It also drives the robot
base link to move through the found path.

### slam_gmapping
[slam_gmapping](http://wiki.ros.org/slam_gmapping) is a widely used ROS package for mapping purposes.
It uses robot laser scanner data along with robot base frame position in order to find robot in the map.
Since there is map file provided in this package, it generates the map in an online manner.

### finite state machin 
The main part of software architucture, defines the states and transitions for the finite state
machine of the robot behaviour. It is implemented using the method presented in [smach](http://wiki.ros.org/smach)
It also uses ``topological_map.py`` helper script which is based on [aRMOR](https://github.com/EmaroLab/armor)
to update the ontology while the process is running.

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/fsm.png" width="300" title="fsm">
</p>

### armor_service
It is used for manipulating the ontology and getting information in a query throgh ``finite_state_machine`` node.

## Temporal Diagram (UML Sequence Diagram)
The following figure represents the UML sequence diagram of this package. In the initial state, user launches the nodes,
``finite_state_machine`` calls ``move_arm`` service to initialze robot arm movement using ``moveit``. Markers 
get scanned by robot camera and for each marker ``image_id`` would be detected. ``finite_state_machine`` requests ``room_info``
by sending room id and build semantic map in the ontology. Then ``base_movement_state`` gets true by ``finite_state_machine`` node
for enabling ``robot-states`` node to simulate battery consumption and target room position would be sent to ``moveit``
node to find the path and move the robot base. Once the robot gets to target position ``base_movement_state`` gets false
and robot starts exploring the room like the beginning of the sequence. 

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/sequence.png" width="820" title="sequence_diagram">
</p>

## Usage
### Installation
* This package is based on [aRMOR](https://github.com/EmaroLab/armor) it has to be installed as it is described
in the provided link as a pre-condition for running this package.

* It is also depended on [smach](http://wiki.ros.org/smach), it can be installed using the following commands:

```bashscript
$ sudo apt-get install ros-<distro>-executive-smach*
```
```bashscript
$ sudo apt-get install ros-<distro>-smach-viewer
```

* For image processing part, the [aruco_ros](https://github.com/CarmineD8/aruco_ros) and [cv_bridge](http://wiki.ros.org/cv_bridge) packages have to be cloned and setup.

* Regarding the navigation part, [slam_gmapping](http://wiki.ros.org/slam_gmapping)  and [move_base](http://wiki.ros.org/move_base) have to be installed .

* Finally, the [MoveIt](https://moveit.ros.org/) package has to be installed for robot joints trajectory planning and control.


Once the dependencies are met, the package can be installed as it follows:

```bashscript
$ mkdir -p catkin_ws/src
```
```bashscript
$ cd catkin_ws/src
```
```bashscript
$ git clone https://github.com/aliy98/ros-moveit-opencv-ontology
```
```bashscript
$ cd ..
```
```bashscript
$ source /opt/ros/<distro>/setup.bash
```
```bashscript
$ catkin_make
```

### Running
In order to initialize the software architucture along with the finite state machine representation, run the following command.

```bashscript
$ source devel/setup.bash
```
```bashscript
$ roslaunch assignment2 assignment.launch
```
Then, in another terminal, initialize ``finite_state_machine`` node using this command:
```bashscript
$ rosrun assignment2 finite_state_machine.py
```

Here is the result for the first state while the robot scans the markers:

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/build_map.gif" title="build_map">
</p>

Once the robot builds the semantic map, it would move to target room as it is shown below:

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/move_to_target.gif" title="move_to_target">
</p>

When it reaches the target, the state will change to explore room:

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/explore_room.gif" title="explore_room">
</p>

While the robot moves to target, if the battery gets lower than threshold, it will move to charger:

<p align="center">
<img src="https://github.com/aliy98/ros-moveit-opencv-ontology/blob/master/docs/diagrams/move_to_charger.gif title="move_to_charger">
</p>

## Working Hypothesis and Environment

1. System's Features: The robot URDF which is used in this package was able to detect the markers with a camera mounted on a 5 dof manipulator. 
Additionally, it could find the best path to the target point while it generated the map in an online simultanious manner.

2. System's Limitations: Battery level is the main limitation in this experiment, although it has chosen high enough to make the robot able to 
patrol the environment more conveniently.

3. Possible Technical Improvements: The robot could have been designed in such a way that there was a balance between the base and manipulator 
to avoid swinging in real case scenario. Of course, the ontology part could have been more comprehensive considering the power of PELLET reasoner.

## Authors and Contacts
- Ali Yousefi
- email: aliyousefi98@outlook.com 