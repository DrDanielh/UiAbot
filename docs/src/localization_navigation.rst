:tocdepth: 2

Localization and Navigation
==============================
The main goal of the project is to make the UiAbot move autonomously from point A to point B, while dynamically avoid obstacles along the path.
To achieve this goal we have implemented the `Nav2 <https://navigation.ros.org/>`_ software stack. This includes a lot of sub-packages
and tools thak make it possible to create an autonomous mobile robot. The expected inputs to Nav2 are TF transformations conforming to 
`REP-105 <https://www.ros.org/reps/rep-0105.html>`_, a map source if utilizing the Static Costmap Layer, a BT XML file, and any relevant sensor 
data sources. It will then provide valid velocity commands in the form of `geometry_msgs/Twist <http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html>`_ 
messages to the ``/cmd_vel`` topic to close the loop.

.. _localization_and_navigation localization_and_navigation_diagram:

.. figure:: fig/motion_planning_and_navigation.drawio.svg
    :width: 1000
    :align: center

    Figure: Motion Planning and Navigation diagram.

The Nav2 stack has tools to:

- Load, serve, and store maps (Map Server)
- Localize the robot on the map (AMCL)
- Plan a path from A to B around obstacles (Nav2 Planner)
- Control the robot as it follows the path (Nav2 Controller)
- Smooth path plans to be more continuous and feasible (Nav2 Smoother)
- Convert sensor data into a costmap representation of the world (Nav2 Costmap 2D)
- Build complicated robot behaviors using behavior trees (Nav2 Behavior Trees and BT Navigator)
- Compute recovery behaviors in case of failure (Nav2 Recoveries)
- Follow sequential waypoints (Nav2 Waypoint Follower)
- Manage the lifecycle and watchdog for the servers (Nav2 Lifecycle Manager)
- Plugins to enable your own custom algorithms and behaviors (Nav2 Core)

Localization in predefined map
------------------------------
In contradiction to :ref:`SLAM <localization_mapping slam>`, where the map is created simultaneously, localization in Nav2 relies on a predefined
map getting parsed in upon launch. The predefined map of the environment consists of a *.pgm* containing a gray-scale of the map itself 
and a *.yaml* file with map parameters. These files are created when you save a map during a mapping routine using e.g. SLAM.

The package utilizes `AMCL <http://wiki.ros.org/amcl>`_, which is a probabilistic localization system for a robot moving in 2D. AMCL, or Adaptive
Monte Carlo Localization, uses a particle filter to track the pose of a robot against a known map based on the received range measurements from 
e.g. the LiDAR.

When AMCL has found a valid localization of the robot it sends the pose as a TF of ``base_link`` relative to ``map``. The ``base_link->odom`` TF
will be kept based on the current odometry, meaning the ``odom`` frame will follow along localization. If the pose of the robot in 
the map is somewhat known, you can set an initial pose estimate in Rviz2 to help AMCL initially localize more correctly.

.. note::
    Command to launch the ``localization_launch.py`` nodes:

    .. code-block:: bash

        ros2 launch nav2_bringup localization_launch.py use_sim_time:=False map:=<map_file_path> params_file:=<nav2_params_file_path> 

    The launch argument ``use_sim_time`` is only set to true if using simualted environments, such as Gazebo.

Navigation
----------

The navigation of the robot is achieved thorugh numerous plugins and tools. One of these plugins is the ``NavFn`` planner, which computes the shortest 
path from a pose to a goal pose using A* or Dijkstra´s algorithm. UiAbot uses the Dijkstra. It assumes a circular robot (or a robot that can be 
approximated as circular for the purposes of global path planning) and operates on a weighted costmap, which is a 2D grid-based map for environmental 
representations a number of sensor processing plugins. It is used in the planner and controller servers for creating space to check for collisions or 
higher cost areas to negotiate around.

.. figure:: res/localization_navigation.gif
    :width: 1000
    :align: center

    Figure: A gif showing the UiAbot performing Localization and Navigation.

The gif above demonstrates the UiAbot while it performs a localization in a static map as well as navigation trough multiple waypoints. Notice the local
and global costmap as it moves through the trajectory. You can also see the green particles from AMCL dynamically changing based on how certain the  
localized pose is.

.. note::
    Command to launch the ``navigation_launch.py`` nodes:

    .. code-block:: bash

        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=<nav2_params_file_path> 

    The launch argument ``use_sim_time`` is only set to true if using simualted environments, such as Gazebo.

SLAM and Navigation
-------------------

It is also possible to do mapping with the ``slam_toolbox`` and navigation at the same time. This will not require a static map, given that the map will
be created while navigating. When running SLAM, the localization nodes from Nav2 should not be running simultaneously.

.. figure:: res/slam_navigation.gif
    :width: 1000
    :align: center

    Figure: A gif showing the UiAbot performing SLAM and Navigation.


Nav2 parameters
---------------
The used parameter file for the Nav2 package is based on the default parameters. If the package is installed using ``apt``, as shown in :ref:`installation_setup`,
then the default parameter file is found in ``/opt/ros/galactic/share/nav2_bringup/params/nav2_params.yml``.

A copy of this file is placed in ``/home/jetson/uiabot_ws/src/uiabot/params/``, which then gets parsed into the launch files for both nav2 localization and navigation. The
modified parameters is listed below.

.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        odom_topic: /odometry/filtered        # /odom

    controller_server:
      ros_parameters:
        FollowPath:
          acc_lim_x: 2.0                      # 2.5
          acc_lim_theta: 1.0                  # 3.2

          decel_lim_x: -2.0                   # -2.5
          decel_lim_theta: -1.0               # -3.2

    local_costmap:
      local_costmap:
        ros_parameters:
          # robot_radius: 0.22
          footprint: "[ [0.1, 0.25], [0.1, -0.25], [-0.35, -0.25], [-0.35, 0.25] ]"

    global_costmap:
      global_costmap:
        ros_parameters:
          # robot_radius: 0.22
          footprint: "[ [0.1, 0.25], [0.1, -0.25], [-0.35, -0.25], [-0.35, 0.25] ]"

The default ``odom_topic`` of the *behaviour-tree navigator* was replaced to the topic that the ``ekf_filter_node`` publishes the fused odometry to. Additionally,
acceleration and deceleration limits was decreased to make a more gentle start. All velocities are kept as default.

The local and global costmap parameters are modified with a rectangular footprint of the robot instead of the default circle. The geometry of this footprint
is given in the ``base_link`` frame.