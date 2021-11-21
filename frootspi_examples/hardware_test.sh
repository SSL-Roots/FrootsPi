#!/bin/sh

# ドリブラー動作テスト
echo "ドリブラーテスト"
ros2 topic pub --once /dribble_power frootspi_msgs/msg/DribblePower "{power : 1.0}"
ros2 topic pub --once /dribble_power frootspi_msgs/msg/DribblePower "{power : 0.0}"

# ホイール動作テスト
echo "ホイールテスト"
ros2 topic pub --once /target_wheel_velocities frootspi_msgs/msg/WheelVelocities "{front_left: 0.0, front_right: 5.0, back_center: 0.0}"
ros2 topic pub --once /target_wheel_velocities frootspi_msgs/msg/WheelVelocities "{front_left: 0.0, front_right: 0.0, back_center: 0.0}"
ros2 topic pub --once /target_wheel_velocities frootspi_msgs/msg/WheelVelocities "{front_left: 5.0, front_right: 0.0, back_center: 0.0}"
ros2 topic pub --once /target_wheel_velocities frootspi_msgs/msg/WheelVelocities "{front_left: 0.0, front_right: 0.0, back_center: 0.0}"
ros2 topic pub --once /target_wheel_velocities frootspi_msgs/msg/WheelVelocities "{front_left: 0.0, front_right: 0.0, back_center: 5.0}"
ros2 topic pub --once /target_wheel_velocities frootspi_msgs/msg/WheelVelocities "{front_left: 0.0, front_right: 0.0, back_center: 0.0}"

# キッカーテスト
ros2 service call /set_kicker_charging frootspi_msgs/srv/SetKickerCharging "{start_charging: true}"
ros2 service call /set_kicker_charging frootspi_msgs/srv/SetKickerCharging "{start_charging: false}"
ros2 service call /kick frootspi_msgs/srv/Kick "{kick_type: 3, kick_power: 1.0}"

echo "放電待ち 10秒待機してください..."
sleep 10

# ボールセンサーテスト
echo "ボールセンサーテスト ボールセンサーの反応を確かめてください"
timeout -sKILL 10 ros2 topic echo /ball_detection # 5秒で止まる