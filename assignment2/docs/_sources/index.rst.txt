Welcome to ros-moveit-opencv-ontology's documentation!
======================================================
.. toctree::
   :maxdepth: 2
   :caption: Contents:

Introduction
=============
This package is an experiment to use a topological map ontology for controling a robot using ROS.
The ontology consists of an indoor environment with multiple rooms and a mobile robot.

.. image:: diagrams/topological_map.jpg
  :width: 300
  :align: center
  :alt: topological_map

The robot starts in room E and by scanning the provided markers, it receives the information to build the
semantic map, i.e., the name and center position of each room and the connections between them.

Once the semantic map is built, robot has to start moving among the rooms with the policy that each
room that has not been visited for a long time, would be selected as the target room. Everytime the robot
gets to the target room, it has to scan the room environment as it did for the first time with the markers.

When the robot battery is low, it goes to the charger which is placed in room E, 
sand wait for some times before to start again with the above behavior.

Contents
==========
.. toctree::
   :maxdepth: 2
   
   simulation_environment
   software_architecture
   temporal_diagram
   scripts
   src_files
   usage
   working_hypothesis
   
Indices and tables
==================
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
