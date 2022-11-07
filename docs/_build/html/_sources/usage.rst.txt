Usage
=====

Installation
------------
This package is based on `aRMOR <https://github.com/EmaroLab/armor>`_, it has to be installed as it is described
in the provided link as a pre-condition for running this package. 

It is also depended on `smach <http://wiki.ros.org/smach>`_, it can be installed using the following commands:

.. code-block:: console

   $ sudo apt-get install ros-<distro>-executive-smach*
   $ sudo apt-get install ros-<distro>-smach-viewer

Once the dependencies are met, the package can be installed as it follows:

.. code-block:: console

   $ mkdir -p catkin_ws/src
   $ cd catkin_ws/src
   $ git clone https://github.com/aliy98/ontological_robot_control
   $ cd ..
   $ source /opt/ros/<distro>/setup.bash
   $ catkin_make

Running
--------

In order to initialize the software architucture along with the finite state machine representation, run the following command.

.. code-block:: console

   $ source devel/setup.bash
   $ roslaunch ontological_robot_control topological_map.launch

Here is the final result of launching this package:

.. image:: diagrams/result.gif
  :width: 800
  :align: center
  :alt: result

