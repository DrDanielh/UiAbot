Project Description
===================

This is the main documentation for the project in MAS514 at the University of Agder (UiA). In which the goal is to drive a mobile robot autonomously using the ROS 2 robotics framework. The course gives a brief introduction to several topics related to robotics and instrumentation, including motion control, perception, localization and mapping, and motion planning and navigation.

Each group is given a differential drive robot, called UiAbot, inspired by turtlebot. The UiAbot is equipped with sensors such as wheel encoders, a LiDAR, and an IMU. A Jetson Nano with Ubuntu 20.04 is used for sensor data processing and control.

In a subsequent project, the Jetson Nano setup was upgraded to a Jetson Orin Nano running Ubuntu 22.04. This upgrade provided improved performance and support for advanced features, benefiting tasks like data processing and sensor integration.Additionally, the system is compatible with both Ubuntu 22.04 and 24.04 on the PC, ensuring better flexibility and compatibility for the development environment.

*Project by*: Martin Mæland, Tarjei Skotterud, and Martin Dahlseng Hermansen

*Supervisor*: `Daniel Hagen <https://www.uia.no/en/kk/profile/danielh>`_

.. figure:: fig/uiabot.svg
    :width: 800
    :align: center

    Figure: The UiAbot with its default AMR setup equipped with an SBC (1), a spinning LiDAR (2), and an IMU (3)
    
Design
--------------
The UiAbot was designed with a focus on flexibility and ease of integration, supporting ROS 2, robotic manipulators, and SBCs like Raspberry Pi and NVIDIA Jetson. It includes sensors such as cameras, encoders, LiDARs, and IMUs for enhanced perception. As shown in Fig. 1, the default setup features an NVIDIA Jetson Nano (1), wheel encoders, a LiDAR (2), and an IMU (3), enabling autonomous navigation. The chassis (4), inspired by TurtleBot3’s modular 'waffle plate' design, uses four full and two half-waffle plates. Structural support comes from the front motor-encoder-gearbox assembly (5) and rear spherical support wheels (6). The left side plate (7) contains the power button, battery status indicator, and charging port, while the right side plate (8) includes an emergency stop button for safety.

Acknowledgment
--------------
Inspired by Turtlebot3, Prof. Dr. Daniel Hagen developed the UiAbot from 2021 to 2022 in collaboration with Mechatronics students. Funding for the necessary equipment and development of this mobile robot platform was generously provided by the Mechatronics Section at the University of Agder.    