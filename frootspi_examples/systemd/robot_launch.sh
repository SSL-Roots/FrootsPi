#!/bin/bash

ENVFILE=/home/ubuntu/ros2_ws/install/setup.bash

if [ -f ${ENVFILE} ]; then
    #環境変数読み込み
    echo "Loading ROS2 Env..."
    source ${ENVFILE}

    echo "ROS2 Launching..."
    #roslaunch実行
    exec ros2 launch frootspi_examples robot.launch.py
else
    echo "There is no ${ENVFILE}"
fi