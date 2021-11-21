#!/bin/bash

ENVFILE=/home/ubuntu/ros2_ws/install/setup.bash
ROS_DOMAIN_ID_VALUE=22

if [ -f ${ENVFILE} ]; then
    #環境変数読み込み
    echo "Loading ROS2 Env..."
    source ${ENVFILE}
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}

    # IPアドレスからロボットIDを取得
    # IPアドレスの下2ケタがIDと対応している
    IFACE="wlan0"
    IP_ADDRESS=$(/sbin/ip -f inet -o addr show "${IFACE}" | cut -d\  -f 7 | cut -d/ -f 1)
    ROBOT_ID=`echo ${IP_ADDRESS: -2:2} | sed -e 's/^0//g'`

    echo "ROS2 Launching..."
    #roslaunch実行
    exec ros2 launch frootspi_examples robot.launch.py id:=${ROBOT_ID}
else
    echo "There is no ${ENVFILE}"
fi