.. _installation_setup:

Installation and Setup
======================

This page will walk you through the required steps to get the UiAbot set up with the correct software and settings.

.. warning::

    It is important to follow each step carefully to avoid compatability problems later, and to understand the commands.

Operating System
----------------

The used OS on the jetson nano is the modifyed jetpack from `QEngineering <https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image/>`_.
This 32GB image has ubuntu 20.04 as well as some common packages and tools for vision, machine learning, etc. It is downloaded following
the instructions on their github page and flashed to a 64GB micro SD-card using `balena etcher <https://www.balena.io/etcher/>`_. 
Once installed and booted on the jetson, the disk partition is expanded to utilize the full 64GB storage using `gparted <https://gparted.org/>`_.

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
    | **Password:** mas514?group5

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

1. Install the following third-party software.

   * ROS 2 Galactic `installation guide <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#ubuntu-debian>`__.
   * odrivetool `installation guide <https://docs.odriverobotics.com/v/latest/getting-started.html#install-odrivetool>`__.

2. Install the following third-party ROS 2 packages.

   * ``rplidar_ros`` (`src <https://github.com/Slamtec/rplidar_ros/tree/ros2>`__)

       .. code:: bash

           sudo apt install ros-galactic-rplidar=3.2.4-1focal.20220730.092525 -y

   * ``robot_localization`` (`src <https://github.com/cra-ros-pkg/robot_localization/tree/galactic-devel>`__)

       .. code:: bash

           sudo apt install ros-galactic-robot-localization=2.0.2-1focal.20220730.023610 -y

   * ``slam_toolbox`` (`src <https://github.com/SteveMacenski/slam_toolbox/tree/galactic>`__)

       .. code:: bash

           sudo apt install ros-galactic-slam-toolbox=2.5.1-1focal.20220730.084110 -y

   * ``nav2`` (`src <https://github.com/ros-planning/navigation2/tree/galactic>`__)

       .. code:: bash

           sudo apt install ros-galactic-navigation2=1.0.12-1focal.20220730.095919 -y
           sudo apt install ros-galactic-nav2-bringup=1.0.12-1focal.20220730.095951 -y

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

        source /opt/ros/galactic/setup.bash

3. Build workspace.
    
    .. code:: bash

        colcon build 