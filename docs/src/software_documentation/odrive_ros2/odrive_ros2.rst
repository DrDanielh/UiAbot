:tocdepth: 1

.. _odrive_ros2_pkg odrive_ros2:

odrive-ros2
===========

.. _odrive_ros2_pkg odrive_ros2_diagram:

.. figure:: ../../fig/odrive_ros2_pkg_odrive_ros2.drawio.svg
    :width: 1000
    :align: center

    Figure: odrive_ros2 node diagram.

Summary
-------
The ``odrive_ros2`` enables the user to request ODrive axis state changes, publish velocity references for both axis, and subscribe to the position and velocity feedback from the encoders on both axis.

Usage
-----

Run:

.. code-block:: bash

    ros2 run odrive_ros2 odrive_ros2

Interfaces
----------

Publishers
^^^^^^^^^^
============================         ============================           =============================
Topic                                Message type                           Description
============================         ============================           =============================
axis0/vel                            std_msgs/Float32                       Axis0 velocity, in rad/s.           
axis1/vel                            std_msgs/Float32                       Axis1 velocity, in rad/s.
============================         ============================           =============================

Subscribers
^^^^^^^^^^^
============================         ============================           =============================
Topic                                Message type                           Description
============================         ============================           =============================
axis0/vel_ref                        std_msgs/Float32                       Axis0 velocity reference, in rad/s.           
axis1/vel_ref                        std_msgs/Float32                       Axis1 velocity reference, in rad/s.
============================         ============================           =============================

Services
^^^^^^^^^^^
============================         ============================           =============================
Topic                                Message type                           Description
============================         ============================           =============================
request_state                        odrive_interfaces/AxisState            Set ODrive axis control state.           
============================         ============================           =============================