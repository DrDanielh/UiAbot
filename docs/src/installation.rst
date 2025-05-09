.. _installation_setup:

Installation and Setup
======================

This page will walk you through the required steps to get the UiAbot set up with the correct software and settings.

.. warning::

    It is important to follow each step carefully to avoid compatability problems later, and to understand the commands.

Operating System
----------------

The operating system utilized on the Jetson Nano is a standard JetPack image installed via `NVIDIA's SDK Manager <https://developer.nvidia.com/sdk-manager>`_. This installation includes Ubuntu 22.04 along with the comprehensive NVIDIA development toolkit and essential packages for computer vision, machine learning, and artificial intelligence applications.

Setting up Wi-Fi
----------------

1. Turn the UiAbot on, and connect to keyboard, mouse, and monitor.
2. On the **jetson** log in to the desired wi-fi, and check if your internet connection.

    .. code:: bash

        ping google.com

    If you do not get this response, you are not connected to the internet.

    .. code:: bash

        jetson@jetson:~$ ping google.com
        PING google.com (142.250.74.110) 56(84) bytes of data.
        64 bytes from arn11s10-in-f14.1e100.net (142.250.74.110): icmp_seq=1 ttl=54 time=28.6 ms
        64 bytes from arn11s10-in-f14.1e100.net (142.250.74.110): icmp_seq=2 ttl=54 time=33.5 ms
        64 bytes from arn11s10-in-f14.1e100.net (142.250.74.110): icmp_seq=3 ttl=54 time=32.8 ms

3. Print your ip-address, and write it down. It will be needed later.

    .. code:: bash

        ip -br a

    It should return something like this.

    .. code:: bash

        jetson@jetson:~$ ip -br a
        lo               UNKNOWN        127.0.0.1/8 ::1/128 
        enp2s0f0         DOWN           
        enp5s0           DOWN           
        wlp3s0           UP             <ip_address>/24 fe80::f9d3:4190:d73:2118/64
                                               
4. Now, unplug the keyboard, mouse, and monitor.

Connecting to UiAbot (SSH)
--------------------------

1. On your **pc**, make sure you are on the same wi-fi as the UiAbot from the previous section.
2. Connect to the UiAbot with SSH.

    .. code:: bash

        ssh jetson@<ip_address>

3. Use these credentials to log in.

    | **Username:** jetson
    | **Password:** jetson

Set up ``.bashrc``
------------------

1. Add usb access.
   
    .. code:: bash

        echo "sudo chmod 666 /dev/ttyUSB0" >> /home/jetson/.bashrc

2. Add ``ROS_DOMAIN_ID``. Change ``5`` to your group number, if applicable. Remember this variable.

    .. code:: bash

        echo "export ROS_DOMAIN_ID=5" >> /home/jetson/.bashrc

3. Install and add ``sl``.

    .. code:: bash

        sudo apt install sl -y && echo "sl" >> /home/jetson/.bashrc

4. Source to update changes.

    .. code:: bash

        source /home/jetson/.bashrc


Install third-party software
----------------------------

1. Install the following third-party software:

   On Jetson:
      * ROS 2 Humble: `Installation Guide <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`__
      * odrivetool: `Installation Guide <https://docs.odriverobotics.com/v/latest/getting-started.html#install-odrivetool>`__

   On PC:
      * ROS 2 Humble: `Installation Guide <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`__ (Ubuntu 22.04 PC)
      * ROS 2 Jazzy: `Installation Guide <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>`__ (Ubuntu 24.04 PC)

2. Install the following third-party ROS 2 packages.

   * ``rplidar_ros`` (`src <https://github.com/Slamtec/rplidar_ros/tree/ros2>`__)

       .. code:: bash

           sudo apt install ros-humble-rplidar-ros -y

   * ``robot_localization`` (`src <https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel>`__)

       .. code:: bash

           sudo apt install ros-humble-robot-localization -y

   * ``slam_toolbox`` (`src <https://github.com/SteveMacenski/slam_toolbox/tree/humble>`__)

       .. code:: bash

           sudo apt install ros-humble-slam-toolbox -y

   * ``nav2`` (`src <https://github.com/ros-planning/navigation2/tree/humble>`__)

       .. code:: bash

           sudo apt install ros-humble-navigation2 -y
           sudo apt install ros-humble-nav2-bringup -y

Create workspace
------------------------

1. Create a workspace in the home folder and call it ``uiabot_ws``.

    .. code:: bash

       cd /home/jetson
       mkdir -p uiabot_ws/src 

The ``jetson`` directory should now look like this.
    
    .. code::

        jetson
        ├── Desktop
        ├── Documents
        ├── Downloads          
        ├── Pictures          
        └── uiabot_ws <-- this is our workspace       
            └── src   <-- all our ROS 2 packages goes here  

Install developed software
--------------------------

1. Enter the workspace ``src`` directory.

    .. code:: bash

        cd /home/jetson/uiabot_ws/src

2. Download the following packages.

   * ``bno055_i2c_ros2`` (`src <https://github.com/DrDanielh/bno055-i2c-ros2>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/bno055-i2c-ros2.git -b v1.0.0

   * ``odrive_ros2`` (`src <https://github.com/DrDanielh/odrive-ros2.git>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/odrive-ros2.git -b v1.0.0

   * ``odrive_interfaces`` (`src <https://github.com/DrDanielh/odrive-interfaces>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/odrive-interfaces.git -b v1.0.0

   * ``uiabot`` (`src <https://github.com/DrDanielh/uiabot-ros2>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/uiabot-ros2.git -b v1.0.0

The ``src`` directory should now look like this.

    .. code::

        src
        ├── bno055-i2c-ros2
        ├── odrive-interfaces
        ├── odrive-ros2          
        └── uiabot-ros2

Build the workspace
-------------------

1. Return to the workspace root. 

    .. code:: bash

        cd /home/jetson/uiabot_ws

2. Source the ROS 2 installation.

    .. code:: bash

        source /opt/ros/humble/setup.bash

3. Build workspace.
    
    .. code:: bash

        colcon build 