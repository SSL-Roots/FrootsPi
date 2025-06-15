# FrootsPi クイックスタートガイド

## はじめに

このガイドでは、FrootsPiシステムの基本的な使用方法を説明します。

## 必要なもの

- FrootsPi基板
- Raspberry Pi 4
- Ubuntu 20.04
- ROS 2 Foxy

## セットアップ手順

### 1. システムインストール

[README.md](../README.md#installation)の手順に従って、FrootsPiをインストールしてください。

### 2. 設定ファイルの準備

```bash
# ロボット設定ファイルをホームディレクトリに作成
cat > ~/robot_config.yaml << EOF
robot_id: 0
# その他の設定...
EOF
```

### 3. システム起動

```bash
# pigpioデーモンの起動
sudo pigpiod -s 1

# ROS環境のセットアップ
source ~/ros2_ws/install/setup.bash

# ロボットシステムの起動
ros2 launch frootspi_examples robot.launch.py
```

## 基本的な使用方法

### ジョイスティック制御

```bash
# ジョイスティック制御の起動
ros2 launch frootspi_examples joycon.launch.py

# 別ターミナルでロボット起動
ros2 launch frootspi_examples robot.launch.py
```

### 直接コマンド送信

```bash
# 前進
ros2 topic pub /command consai_frootspi_msgs/msg/RobotCommand \
  "{velocity_x: 1.0, velocity_y: 0.0, velocity_theta: 0.0}"

# キック準備（充電）
ros2 service call /set_kicker_charging frootspi_msgs/srv/SetKickerCharging \
  "{start_charging: true}"

# キック実行
ros2 topic pub /command consai_frootspi_msgs/msg/RobotCommand \
  "{velocity_x: 0.0, velocity_y: 0.0, velocity_theta: 0.0, kick_power: 1.0}"
```

## トラブルシューティング

### よくある問題

1. **CANネットワークエラー**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```

2. **pigpioデーモンが起動しない**
   ```bash
   sudo killall pigpiod
   sudo pigpiod -s 1
   ```

3. **センサーが反応しない**
   - GPIO設定ファイルを確認
   - ハードウェア接続を確認

### ログの確認

```bash
# システムログの確認
ros2 topic echo /rosout

# 個別モジュールの状態確認
ros2 node list
ros2 topic list
```

## さらなる情報

- [システムアーキテクチャ](architecture.md): 詳細なシステム構成
- [モジュール詳細](modules/): 各モジュールの詳細な説明
- [運用マニュアル](../operation.md): ハードウェア操作方法