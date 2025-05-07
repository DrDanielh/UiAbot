Using the UiAbot
==================

This page contains guides for using the UiAbot with the provided software. These guides are based on the implementation done previosly, and expects that all software is installed and set up according to :ref:`installation_setup`.

Regardless of the guide, remember to always source ROS 2 and the built ``uiabot_ws`` workspace. This is important to do on both the **jetson**, and the **pc** that is going to visualize and command the UiAbot. If this is not done, there will be issues with showing the UiAbot mesh files in RViz.

.. code:: bash

    source /opt/ros/humble/setup.bash
    source /home/jetson/uiabot_ws/install/local_setup.bash

(Optinal)
If you are using Ubuntu 24.04 on your PC, source the following setup files:

.. code:: bash

    source /opt/ros/jazzy/setup.bash

Automatic Sourcing
------------------
To make the process of sourcing the ROS 2 and workspace setup files easier, you can add the commands to your shell configuration file.
This will ensure that the required environments are sourced automatically.

.. code:: bash

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source /home/jetson/uiabot_ws/install/local_setup.bash" >> ~/.bashrc
    source ~/.bashrc

Driving remotely with keyboard
------------------------------

This guide launches all nodes required to drive the UiAbot remotely using the pc keyboard.

1. Run the following command on the **jetson** to launch the ``control`` and ``odrive_ros2`` nodes:

    .. code:: bash

        ros2 launch uiabot teleop.launch.py

2. Run the following command on the **pc** to launch the ``teleop_twist_keyboard`` node:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

Visualizing mechanical odometry, LiDAR, and IMU
-----------------------------------------------

This guide launches all nodes required to drive the UiAbot remotely using the pc keyboard, in addition to the sensor nodes for the LiDAR and IMU. It also launches the mechanical odometry node which calculates the robot position relative to the intial frame. The robot description is also published to the network, which enables the visualization in RViz.

1. Run the following command on the **jetson** to launch the control and sensor nodes:

    .. code:: bash

        ros2 launch uiabot teleop_perception.launch.py

2. Run the following command on the **pc** to launch the ``teleop_twist_keyboard`` node:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

3. Run the following command on the **pc** to launch RViz:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        rviz2

SLAM
----

This guide launches all nodes required to drive the UiAbot remotely using the pc keyboard, and performing SLAM.

1. Run the following command on the **jetson** to launch the control, sensor, and SLAM nodes:

    .. code:: bash

        ros2 launch uiabot teleop_slam.launch.py

2. Run the following command on the **pc** to launch the ``teleop_twist_keyboard`` node:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

3. Run the following command on the **pc** to launch RViz with map visualization:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        ros2 launch nav2_bringup rviz_launch.py

4. When you are finished mapping, run the following command to save the map on **jetson**.

    .. code:: bash

        ros2 run nav2_map_server map_saver_cli -f /home/jetson/uiabot_ws/src/uiabot/map/<map_name>


Localization and navigation
---------------------------

This guide launches all nodes required to drive the UiAbot using nav2 in an existing map.

1. Run the following command on the **jetson** to launch the control, sensor, localization, and navigation nodes. Remember to update the map path to match your map:

    .. code:: bash

        ros2 launch uiabot localization_navigation.launch.py map_path:=/home/jetson/uiabot_ws/src/uiabot/map/map_mezzanine.yaml

2. Run the following command on the **pc** to launch RViz with map visualization:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        ros2 launch nav2_bringup rviz_launch.py

SLAM and navigation
-------------------

This guide launches all nodes required to drive the UiAbot using nav2 in while performing SLAM.

1. Run the following command on the **jetson** to launch the control, sensor, SLAM and navigation nodes:

    .. code:: bash

        ros2 launch uiabot slam_navigation.launch.py

2. Run the following command on the **pc** to launch RViz with map visualization:

    .. code:: bash

        export ROS_DOMAIN_ID=5
        ros2 launch nav2_bringup rviz_launch.py