Installation and Setup
======================

This page will walk you through the required steps to get the UiAbot set up with the correct software and settings.

.. warning::

    It is important to follow each step carefully to avoid compatability problems later, and to understand the commands.

NVIDIA Jetson SBC
----------------

Operating System
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The operating system utilized on the Jetson Nano is a standard JetPack image installed via `NVIDIA's SDK Manager <https://developer.nvidia.com/sdk-manager>`_. This installation includes Ubuntu 22.04 along with the comprehensive NVIDIA development toolkit and essential packages for computer vision, machine learning, and artificial intelligence applications.

Setting up Wi-Fi
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

Remote PC
----------------
Its recomended to use a Ubuntu 22.04 operating system with ROS 2 Humble installed. The installation guide can be found `here <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`__.

Connecting to UiAbot (SSH)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. On your **remote pc**, make sure you are on the same wi-fi as the UiAbot from the previous section.
2. Connect to the UiAbot with SSH.

    .. code:: bash

        ssh jetson@<ip_address>

3. Use these credentials to log in.

    | **Username:** jetson
    | **Password:** jetson

Automatic Sourcing
----------------

Set up ``.bashrc`` on UiAbot
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Add usb access.
   
    .. code:: bash

        echo "sudo chmod 666 /dev/ttyUSB0" >> /home/jetson/.bashrc

2. Add ``ROS_DOMAIN_ID``. Change ``5`` to your uiabot number,

    .. code:: bash

        echo "export ROS_DOMAIN_ID=5" >> /home/jetson/.bashrc # Or use the uiabot # you have!

3. Install and add ``sl``.

    .. code:: bash

        sudo apt install sl -y && echo "sl" >> /home/jetson/.bashrc

4. Automatic Sourcing:

    .. code:: bash

        echo "source /home/jetson/uiabot_ws/install/local_setup.bash" >> ~/.bashrc
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

5. Source to update changes.

    .. code:: bash

        source /home/jetson/.bashrc

Set up ``.bashrc`` on Remote PC
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Automatic Sourcing:
To make the process of sourcing the ROS 2 and workspace setup files easier, you can add the commands to your shell configuration file. This will ensure that the required environments are sourced automatically.
   
    .. code:: bash

        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        echo "source ~/uiabot_ws/install/setup.bash" >> ~/.bashrc
        echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
        echo "export ROS_DOMAIN_ID=5" >> ~/.bashrc # Or use the uiabot # you have!
        source ~/.bashrc 

Install third-party software
----------------------------

1. Install the following third-party software:

   On UiAbot:
      * ROS 2 Humble: `Installation Guide <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`__
      * odrivetool: `Installation Guide <https://docs.odriverobotics.com/v/0.5.6/getting-started.html>`__

   On Remote PC:
      * ROS 2 Humble: `Installation Guide <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`__ (Ubuntu 22.04 PC)
      
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

   * ``Other dependencies``

       .. code:: bash

            sudo apt install ros-humble-xacro
            sudo apt install ros-humble-teleop-twist-joy ros-humble-joy

ROS 2 configuration
------------------------

Create workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Enter the workspace ``src`` directory.

    .. code:: bash

        cd /home/jetson/uiabot_ws/src

2. Download the following packages.

   * ``bno055_i2c_ros2`` (`src <https://github.com/DrDanielh/bno055-i2c-ros2>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/bno055-i2c-ros2.git

   * ``odrive_ros2`` (`src <https://github.com/DrDanielh/odrive-ros2.git>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/odrive-ros2.git

   * ``odrive_interfaces`` (`src <https://github.com/DrDanielh/odrive-interfaces>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/odrive-interfaces.git

   * ``uiabot`` (`src <https://github.com/DrDanielh/uiabot-ros2>`__)

       .. code:: bash

           git clone https://github.com/DrDanielh/uiabot-ros2.git
           
The ``src`` directory should now look like this.

    .. code::

        src
        ├── bno055-i2c-ros2
        ├── odrive-interfaces
        ├── odrive-ros2          
        └── uiabot-ros2

Build the workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Return to the workspace root. 

    .. code:: bash

        cd /home/jetson/uiabot_ws

2. Source the ROS 2 installation.

    .. code:: bash

        source /opt/ros/humble/setup.bash

3. Build workspace.
    
    .. code:: bash

        colcon build 

Configuring ODrive
--------------------

Configuration
^^^^^^^^^^^^^

1. Launch odrivetool

    .. code-block:: bash

        odrivetool

2. Current limit

    .. code-block:: bash

        odrv0.axis0.motor.config.current_lim = 10
        odrv0.axis1.motor.config.current_lim = 10

3. Velocity limit

    .. code-block:: bash

        odrv0.axis0.controller.config.vel_limit = 5
        odrv0.axis1.controller.config.vel_limit = 5

3. Enable brake resistor

    .. code-block:: bash

        odrv0.config.enable_brake_resistor = True

4. Brake restance value

    .. code-block:: bash

        odrv0.config.brake_resistance = 0.05


5. Pole pairs

    .. code-block:: bash

        odrv0.axis0.motor.config.pole_pairs = 7
        odrv0.axis1.motor.config.pole_pairs = 7

6. Torque constant

    .. code-block:: bash

        odrv0.axis0.motor.config.torque_constant = 8.27/270
        odrv0.axis1.motor.config.torque_constant = 8.27/270

6. Calibrate motors

    .. code-block:: bash

        odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
	
7. Velocity control

    .. code-block:: bash

        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

8. Save configuration

    .. code-block:: bash

        odrv0.save_configuration()

Testing and Troubleshooting
^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Request closed-loop control

    .. code-block:: bash

        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    .. code-block:: bash

        odrv0.axis0.controller.input_vel = 1
        odrv0.axis1.controller.input_vel = 1

2. Dump errors

    .. code-block:: bash

        dump_errors(odrv0)

3.Clear errors

    .. code-block:: bash

        odrv0.clear_errors()
