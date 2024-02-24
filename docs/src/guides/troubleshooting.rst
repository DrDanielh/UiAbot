Troubleshooting
===============

1. Check if both the jetson and pc is connected to the same network.
2. Check if the environmental variable ``ROS_DOMAIN_ID`` is set to the same value on both the jetson and pc.
3. Hardware connections.
4. *Ctrl-C* does not always shut down all processes. Make sure they are completely shut down with ``ps aux`` tool, before re-launching nodes or similar.
5. If you get colcon build warning "clock skew detected", run the following command.

    .. code:: bash

        cd /home/jetson/uiabot_ws && rm -rf build install log  && find  . -type f | xargs -n 5 touch

