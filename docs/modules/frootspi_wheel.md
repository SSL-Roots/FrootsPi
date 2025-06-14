# frootspi_wheel モジュール

## 概要

`frootspi_wheel`は、FrootsPiの車輪制御を担当するモジュールです。目標速度（Twist）を受信し、それを3輪オムニホイール用の個別車輪速度に変換して制御します。

## 主な機能

- 目標速度（Twist）から個別車輪速度への変換
- オムニホイール運動学計算
- 車輪速度制御コマンド生成

## インターフェース

### Subscription （受信トピック）
- `target_velocity` (geometry_msgs/Twist): 目標速度（x, y, θ）

### Publication （送信トピック）
- `target_wheel_velocities` (frootspi_msgs/WheelVelocities): 個別車輪目標速度

## 車輪構成

FrootsPiは3輪オムニホイール構成：
- `front_left`: 前左車輪
- `front_right`: 前右車輪  
- `back_center`: 後中央車輪

## 運動学変換

目標速度（vx, vy, ω）から各車輪速度への変換：

```
front_left  = -0.5 * vx + √3/2 * vy + L * ω
front_right = -0.5 * vx - √3/2 * vy + L * ω
back_center =  1.0 * vx + 0.0 * vy + L * ω
```

（L: ロボット中心から車輪までの距離）

## 使用例

```bash
# 前進指令
ros2 topic pub /target_velocity geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 右移動指令
ros2 topic pub /target_velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 回転指令
ros2 topic pub /target_velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# 車輪速度確認
ros2 topic echo /target_wheel_velocities
```