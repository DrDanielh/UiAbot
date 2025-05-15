Overview
===================

This is the main documentation for the UiAbot Autonomous Mobile Robot (AMR) platform developed at the University of Agder (UiA). The goal of the UiAbot is to introduce STEM students to programming, implementing, and autonomously driving a mobile robot using the Robotic Operating System framework (ROS 2). This documentation provides a brief introduction to several topics related to autonomous robotics, including motion control, perception, localization and mapping (SLAM), and motion planning and navigation.

The initial Single Board Computer (SBC) setup, which used the NVIDIA Jetson Nano running Ubuntu 20.04 (Jetpack 5) with ROS 2 Galactic, has been upgraded to a Jetson Orin Nano running Ubuntu 22.04 (Jetpack 6) with ROS 2 Humble. This upgrade enhances performance and enables support for advanced features, benefiting tasks such as data processing and sensor integration.

.. figure:: fig/uiabot.svg
    :width: 800
    :align: center

    Figure: The UiAbot with its default AMR setup, equipped with an SBC (1), a spinning LiDAR (2), and an IMU (3).
    
Design
--------------
The UiAbot was designed with a focus on flexibility and ease of integration, supporting ROS 2, robotic manipulators, and SBCs like Raspberry Pi and NVIDIA Jetson. It includes sensors such as cameras, encoders, LiDARs, and IMUs for enhanced perception. As shown in Fig. 1, the default setup features an NVIDIA Jetson Nano (1), wheel encoders, a LiDAR (2), and an IMU (3), enabling autonomous navigation. The chassis (4), inspired by TurtleBot3’s modular 'waffle plate' design, uses four full and two half-waffle plates. Structural support comes from the front motor-encoder-gearbox assembly (5) and rear spherical support wheels (6). The left side plate (7) contains the power button, battery status indicator, and charging port, while the right side plate (8) includes an emergency stop button for safety.

Acknowledgment
--------------
Inspired by TurtleBot3, Prof. Dr. `Daniel Hagen <https://www.uia.no/en/kk/profile/danielh>`_ developed the UiAbot from 2021 to 2022 in collaboration with Mechatronics students. Funding for the necessary equipment and the development of this mobile robot platform was generously provided by the Mechatronics Section at the University of Agder.