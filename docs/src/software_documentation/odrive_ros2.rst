:tocdepth: 3

.. _odrive_ros2_pkg:

odrive-ros2
===========

=======  =====
Package  odrive_ros2
Git      `git <https://github.com/DrDanielh/odrive-interfaces>`_
Version  1.0.0 
ROS 2    Galactic, Humble
Node(s)  odrive_ros2
=======  =====

Summary
-------
The ``odrive_ros2`` package is a ROS 2 wrapper around the ODrive api. It contains a single node, which has the same name as the package itself ``odrive_ros2``.

Dependencies
------------

* :ref:`odrive_interfaces_pkg`

Usage
-----

Install:

.. code-block:: bash

    git clone https://github.com/DrDanielh/odrive-ros2.git

Build:

.. code-block:: bash

    colcon build --packages-select odrive_ros2

Nodes
-----

.. toctree::
   :hidden:

   odrive_ros2/odrive_ros2

* :doc:`odrive_ros2/odrive_ros2`