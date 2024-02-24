Localization and Mapping
========================

Localization and mapping are important aspects make a robot drive autonomously. Localization is the process in which the robot estimates it's position in a given or generated map. While mapping is the process of generating a map from the current environment. 

The figure below illustrates how localization and mapping is implemented by extending on the software from :ref:`perception`.

.. _localization_mapping localization_mapping_diagram:

.. figure:: fig/localization_mapping.drawio.svg
    :width: 1000
    :align: center

    Figure: Localization and mapping communication diagram.

Odometry
--------
There are several approaches to localization, one of the most common being odometry. The odometry uses motion sensors to estimate the robot's current position in reference to an initial frame, usually refered to as the odometry- or odom-frame. Sensors on the UiAbot that can be used for odometry is: wheel encoders and IMU.

Mechanical Odometry
^^^^^^^^^^^^^^^^^^^

Localization using encoders is often refered to as mechanical odometry, since it uses encoders to get data from the mechanical parts of the robot. In this project we use the  angular velocity feedback from the wheel encoders. The mechanical odometry uses the :ref:`motion_control forward_kinematics` to calculate the actual robot velocities in the local coordinate frame, by transforming the angular velocities. The robot velocities are then integrated and decomposed to find the robot position and angle with respect to the ``odom`` frame. This is implemented in the ``mechanical_odometry`` node introduced in :ref:`motion_control`. The ``mechanical_odometry`` node publishes the robot position and velocity w.r.t. the odom-frame in a `nav_msgs/Odom <http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html>`_ message on the ``/mechanical_odometry`` topic.

.. note::
    Command to run the ``mechanical_odometry`` node:

    .. code-block:: bash

        ros2 run uiabot mechanical_odometry --ros-args -r use_tf:=true

    Note that the ``use_tf`` parameter is set to ``true``. This is because the ``mechanical_odometry`` node must publish the transform between the ``odom`` frame and robot ``base_link`` frame. This will later be set to ``false`` to avoid multiple transform broadcasters to the same frames.

Sensor Fusion
^^^^^^^^^^^^^

The mechanical odometry is prone to drift because of small inaccuracies in the wheel encoder measurements, which will lead to errors accumulating over time while integrating. Errors will also be introduced if the robot is stuck and the wheels keep spinning, or the robot slips on a smooth surface. This means that the mechanical odometry is accurate only for short periods of time. However, by introducing the measurements from the IMU, we are able to get data about the robot movement, unrelated to the wheel velocities. This way we know if the robot is actually moving or not, by i.e. integrating the IMU's acceleration data.

Combining the data from the mechanical odometry and the IMU is done by using an `extended Kalman Filter <https://en.wikipedia.org/wiki/Extended_Kalman_filter>`_ (EKF). The EKF is a nonlinear version of the regular `Kalman filter <https://en.wikipedia.org/wiki/Kalman_filter>`_, and is considered to be the de facto standard in the theory of nonlinear state estimation. The Kalman filter uses a series of measurements over time to estimate unknown variables that tends to be more accurate than the measurements alone. The Kalman filter also includes statistical noise and other inaccuracies to improve its estimation. On the UiAbot we are using the ``ekf_filter_node`` node from the `robot_localization <http://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html#ekf-localization-node>`_ library to fuse the mechanical odometry and IMU data.

The ``ekf_filter_node`` node uses the parameter file ``ekf_params.yaml`` located in ``/home/jetson/uiabot_ws/src/uiabot/params/ekf_params.yaml``. This file is based on the default parameter file, which is located in ``/opt/ros/galactic/share/robot_localization/params/ekf.yaml``.

The first parameter that is updated is the ``use_sim_time`` parameter. This decides which clock to use for synchronization. Typically this is set to ``true`` when simulating, since then all nodes should synchronize with the simulation clock. However, since we are running on the real robot, we set this parameter to ``false``.

.. code:: yaml

    use_sim_time: false

Since we want to fuse the mechanical odometry and IMU data, we updated the following parameters to define which ROS 2 topics the sensor data is sent on:

.. code:: yaml

    odom0: mechanical_odometry
    imu0: bno055/data

The ``ekf_filter_node`` node also publishes the transform between the ``odom`` and ``base_link`` frames, the frame names are defined by these parameters:

.. code:: yaml

    odom_frame: odom
    base_link_frame: base_link

Since the UiAbot works in a planar environment, we can assume that there will be no movement along the z-axis. We can also assume that the roll and pitch is zero. Based on these assumptions the ``two_d_mode`` parameter is set to true, which effectively ignores the z, roll and pitch information from the sensors. This is beneficial because small imperfections and variations in the ground plane will introduce noise to the sensors.

.. code:: yaml

    two_d_mode: true

The parameter vector for which data to fuse is defined in a vector with this structure:

.. code:: yaml

    # not a real parameter, just to show the structure
    sensorX_config: [x_pos   , y_pos    , z_pos,
                     roll    , pitch    , yaw,
                     x_vel   , y_vel    , z_vel,
                     roll_vel, pitch_vel, yaw_vel,
                     x_accel , y_accel  , z_accel]

The mechanical odometry message contains information about the robot pose and velocities, and while it is tempting to use all this information, you should not. Since the pose is integrated from the velocities, this would introduce duplicate information to the filter. Therefore, it is best to just use the velocity data. If the mechanical odometry provided the position and/or heading from a different sensor than the wheels, that would be another case. The parameter vector defining which data to fuse from the mechanical odometry is then updated to: 

.. code:: yaml

    odom0_config: [false, false, false,
                   false, false, false,
                   true, true, false,
                   false, false, true,
                   false, false, false]

The IMU message contains information about the robot acceleration, orientation, and angular rate. Since the IMU consists of several sensors, the data is not duplicate and we can include both yaw and yaw rate. We ignore the other orientations and angular rates because of the ``two_d_mode`` parameter set earlier. We are setting the y-axis acceleration to false, because we are not expecting any acceleration in that direction. Technically this could be set to true, but because the the acceleration data often is noisy, it is best to ignore it. The x-acceleration is set to true, because this is the driving direction of the UiAbot: 

.. code:: yaml

    imu0_config: [false, false, false,
                  false,  false,  true,
                  false, false, false,
                  false,  false,  true,
                  true,  false,  false]

The last parameter to update is the ``imu0_remove_gravitational_acceleration`` parameter, which should be set to ``true`` if the IMU does not remove the gravitational acceleration. Since our IMU does this, the parameter is set to ``false``.

.. code:: yaml

    imu0_remove_gravitational_acceleration: false

.. note::

    Command to run the ``mechanical_odometry`` node:

    .. code-block:: bash

        ros2 run uiabot mechanical_odometry --ros-args -r use_tf:=false

    Note that the ``use_tf`` parameter is set to ``false``. This is because the ``ekf_filter_node`` node will now publish the transform between the ``odom`` frame and robot ``base_link`` frame.

    Command to run the ``ekf_filter_node`` node:

    .. code-block:: bash

        ros2 run robot_localization ekf_filter_node --ros-args --params-file ./src/uiabot/params/ekf_params.yaml

    
.. _localization_mapping slam:

Simultaneous Localization and Mapping (SLAM)
--------------------------------------------

SLAM is the process of continously creating a map of the current surroundings. For the robot to be able to create a map, it needs data that can be used for such a task, typically vision data. The UiAbot is equipped with a 2D LiDAR, which gives the robot information about the distance to objects surrounding the robot in a plane. The robot must then have to consider which objects that are less likely to move, such as walls. These objects will form the base of the map. After finding these objects in the current robot position, the robot must move to get data from the remaining of the surroundings. While moving the objects previously drawn on the map will also move, because the LiDAR will now register these as closer or further away. This is the part where the robot must localize itself relative to the already drawn map, before it can continue drawing.

For this project we are using the ``slam_toolbox`` developed by Steve Macenski, which is also the default SLAM library used in Nav 2. Link to the source code `here <https://github.com/SteveMacenski/slam_toolbox>`_. The ``slam_toolbox`` updates the UiAbot ``base_link`` relative to the ``map`` frame whenever it localizes itself. This in turn corrects the ``odom`` frame. This can be seen in the Figure below, where the ``map`` and ``odom`` frame initially has the same pose. However, the ``odom`` frame drifts and is corrected over time, while the ``map`` frame is not changed.

.. figure:: res/slam.gif
    :width: 1000
    :align: center

    Figure: A gif showing the UiAbot performing SLAM.

.. note::
    Command to launch the ``online_async_launch.py`` nodes:

    .. code-block:: bash

        ros2 launch slam_toolbox online_async_launch.py
