:tocdepth: 3

.. _uiabot_pkg:

uiabot-ros2
======

=======  =====
Package  uiabot
Git      `git <https://github.com/DrDanielh/uiabot-ros2>`_
Version  1.0.0 
ROS 2    Galactic, Humble
Node(s)  control, mechanical_odometry, imu_tf_viz
=======  =====

Summary
-------

The ``uiabot`` package contains nodes to control the UiAbot, calculate the mechanical odometry, and visualize IMU orientation in RViz. In addition, there are several files used by the UiAbot, such as parameters, robot description, and maps.

Usage
-----

Install:

.. code-block:: bash

    git clone https://github.com/DrDanielh/uiabot-ros2.git

Build:

.. code-block:: bash

    colcon build --packages-select uiabot

Nodes
-----

.. toctree::
   :hidden:

   uiabot/control
   uiabot/mechanical_odometry
   uiabot/imu_tf_viz

* :doc:`uiabot/control`
* :doc:`uiabot/mechanical_odometry`
* :doc:`uiabot/imu_tf_viz`

Package structure
-----------------

::

    uiabot
    ├── include         <- c++ header files
    ├── launch          <- ros 2 launch files
    ├── map             <- saved maps
    ├── params          <- parameter files
    ├── src             <- c++ src files
    ├── urdf            <- uiabot description and meshes
    ├── .gitignore   
    ├── CMakeLists.txt
    ├── README.md          
    └── package.xml

launch
^^^^^^

* ``teleop.launch.py``

  * Control system.
  * The ``teleop_twist_keyboard`` node must be ran on the pc.

* ``teleop_perception.launch.py``

  * Remote control using the computer keyboard.
  * Mechanical odometry.
  * IMU and LiDAR.
  * IMU orientation visualization
  * Robot state publisher.
  * The ``teleop_twist_keyboard`` node must be ran on the pc.

* ``teleop_slam.launch.py``
  
  * Control system.
  * Mechanical odometry.
  * IMU and LiDAR.
  * Sensor fusion (EKF).
  * Robot state publisher.
  * SLAM.
  * The ``teleop_twist_keyboard`` node must be ran on the pc.

* ``localization_navigation.launch.py``
  
  * Control system.
  * Mechanical odometry.
  * IMU and LiDAR.
  * Sensor fusion (EKF).
  * Robot state publisher.
  * Localization and navigation (Nav2).

* ``slam_navigation.launch.py``
  
  * Control system.
  * Mechanical odometry.
  * IMU and LiDAR.
  * Sensor fusion (EKF).
  * Robot state publisher.
  * SLAM.
  * Navigation (Nav2).

map
^^^

* ``map_hallway_c2``
* ``map_mezzanine``
* ``map_mezzanine_2``

params
^^^^^^

* ``ekf_params.yaml``
  
  * Parameters for the ``ekf_filter_node`` node.

* ``nav2_params.yaml``

  * Parameters for the ``nav2`` package.

urdf
^^^^

* ``uiabot.urdf.xacro``

  * Robot description in urdf format with xacro format wrapper.
  * Parsed into the ``robot_state_publisher`` in the launch files. 