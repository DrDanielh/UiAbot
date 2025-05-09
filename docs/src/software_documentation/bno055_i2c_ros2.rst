:tocdepth: 3

.. _bno055_i2c_ros2_pkg:

bno055-i2c-ros2
===============

=======  =====
Package  bno055_i2c_ros2
Git      `git <https://github.com/DrDanielh/bno055-i2c-ros2>`_
Version  1.0.0 
ROS 2    Humble
Node(s)  bno055_i2c_ros2
=======  =====

Summary
-------
This package is originally a ROS(1) package that was re-written to work in ROS2 during this project. The original package and its documentation is 
found `here <https://github.com/dheera/ros-imu-bno055>`_. 

Note that the service calls are not implemented as in the original ROS(1) package.

Usage
-----

Install:

.. code-block:: bash

    git clone https://github.com/DrDanielh/bno055-i2c-ros2.git

Build:

.. code-block:: bash

    colcon build --packages-select bno055_i2c_ros2

Nodes
-----

.. toctree::
   :hidden:

   bno055_i2c_ros2/bno055_i2c_ros2

* :doc:`bno055_i2c_ros2/bno055_i2c_ros2`