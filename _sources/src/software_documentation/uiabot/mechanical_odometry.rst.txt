:tocdepth: 1

.. _uiabot_pkg mechanical_odometry:

mechanical_odometry
===================

.. _ uiabot_pkg_mechanical_odometry_diagram:

.. figure:: ../../fig/uiabot_pkg_mechanical_odometry.drawio.svg
    :width: 1000
    :align: center

    Figure: mechanical_odometry node diagram.

Summary
-------
The `mechanical_odometry` node subscribes to the velocities of each motor, then calculates the odometry based on the geometry of the differential drive robot by using :ref:`motion_control forward_kinematics`. The mechanical odometry is then published to the ``/mechanical_odometry`` topic. The wheel angles are also published to the ``/joint_states`` topic to update the wheel frames. 

Usage
-----

Run:

.. code-block:: bash

    ros2 run uiabot mechanical_odometry

Interfaces
----------

Publishers
^^^^^^^^^^
============================         ============================           =============================
Topic                                Message type                           Description
============================         ============================           =============================
mechanical_odometry                  nav_msgs/Odometry                      Calculated mehcanical odometry.
joint_states                         sensor_msgs/JointState                 Wheels joint states.
============================         ============================           =============================

Subscribers
^^^^^^^^^^^
============================         ============================           =============================
Topic                                Message type                           Description
============================         ============================           =============================
axis0/vel                            std_msgs/Float32                       Axis0 velocity, in rad/s.           
axis1/vel                            std_msgs/Float32                       Axis1 velocity, in rad/s.
============================         ============================           =============================