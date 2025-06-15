# FrootsPi システムアーキテクチャ

## 概要

FrootsPiは、サッカーロボット用のROS 2ベースの制御システムです。複数のモジュールが連携してロボットの動作を制御します。

## システム構成図

```
[外部システム/ジョイスティック]
            ↓ (RobotCommand)
    [frootspi_conductor] ← 統合制御モジュール
            ↓ (各種トピック)
    ┌─────────────────────────────────┐
    ↓                ↓                ↓
[frootspi_wheel] [frootspi_kicker] [frootspi_hardware]
    ↓                ↓                ↓
    車輪制御          キック制御        ハードウェア制御
                                     (センサー、LED、等)
```

## モジュール間通信

### 主要なトピック

- `command` (RobotCommand): 外部からの統合コマンド
- `target_velocity` (Twist): 目標速度
- `dribble_power` (DribblePower): ドリブラーパワー
- `kick_command` (KickCommand): キックコマンド
- `target_wheel_velocities` (WheelVelocities): 車輪個別速度
- `ball_detection` (BallDetection): ボール検出情報

### サービス

- `capacitor_charge_request` (SetBool): キッカー充電要求
- `set_kicker_charging` (SetKickerCharging): キッカー充電制御
- `kick` (Kick): キック実行

## データフロー

1. **コマンド受信**: `frootspi_conductor`が外部からの`RobotCommand`を受信
2. **コマンド分解**: 統合コマンドを各モジュール向けの個別コマンドに分解
3. **モジュール実行**: 各モジュールが担当する機能を実行
4. **ハードウェア制御**: `frootspi_hardware`が実際のハードウェアを制御

## 起動方法

### 完全システム起動
```bash
ros2 launch frootspi_examples robot.launch.py
```

### 個別モジュール起動
```bash
ros2 launch frootspi_examples conductor.launch.py
ros2 launch frootspi_examples hardware.launch.py
```

## 設定ファイル

- `robot_config.yaml`: ロボットID等の基本設定
- `gpio.yaml`: GPIO設定（ハードウェア制御用）

## 依存関係

- ROS 2 Foxy
- frootspi_msgs: 独自メッセージ定義
- consai_frootspi_msgs: 外部システム連携用メッセージ
- pigpio: GPIO制御ライブラリ