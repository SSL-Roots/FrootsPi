# frootspi_conductor モジュール

## 概要

`frootspi_conductor`は、FrootsPiシステムの中央制御モジュールです。外部システムから受信した統合コマンド（RobotCommand）を各ハードウェアモジュール向けの個別コマンドに分解して送信します。

## 主な機能

- 外部システムからのRobotCommand受信
- コマンドの分解と各モジュールへの配信
- キッカー充電管理
- タイムアウト処理

## インターフェース

### Subscription （受信トピック）
- `command` (RobotCommand): 外部システムからの統合ロボットコマンド

### Publication （送信トピック）
- `target_velocity` (geometry_msgs/Twist): 車輪制御への目標速度
- `dribble_power` (frootspi_msgs/DribblePower): ドリブラーパワー設定
- `kick_command` (frootspi_msgs/KickCommand): キックコマンド

### Service Client （呼び出すサービス）
- `capacitor_charge_request` (std_srvs/SetBool): キッカー充電要求

## コマンド処理フロー

1. RobotCommandメッセージ受信
2. 速度成分を`target_velocity`として送信
3. キックパワーが0より大きい場合、`kick_command`送信
4. ドリブルパワーを`dribble_power`として送信
5. 定期的にキッカー充電要求を送信

## 設定パラメータ

- タイムアウト設定
- 充電要求の送信間隔

## 使用例

```bash
# 単体起動
ros2 launch frootspi_examples conductor.launch.py

# コマンド送信例
ros2 topic pub /command consai_frootspi_msgs/msg/RobotCommand \
  "{velocity_x: 1.0, velocity_y: 0.0, velocity_theta: 0.0, dribble_power: 0.5}"
```