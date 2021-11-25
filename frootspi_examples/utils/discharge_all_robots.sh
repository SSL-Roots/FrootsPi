#!/bin/bash

ros2 service call /robot0/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot1/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot2/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot3/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot4/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot5/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot6/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot7/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot8/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot9/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot10/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &
ros2 service call /robot11/kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}" &