# frootspi_kicker モジュール

## 概要

`frootspi_kicker`は、FrootsPiのキッカー機構を制御するモジュールです。キック動作、充電制御、放電処理を管理します。

## 主な機能

- キックコマンド処理（ストレート、チップキック）
- キッカー充電制御
- 放電処理
- 安全機能（タイムアウト、過充電防止）

## インターフェース

### Subscription （受信トピック）
- `kick_command` (frootspi_msgs/KickCommand): キック指令

### Service Server （提供するサービス）
- `kick` (frootspi_msgs/srv/Kick): 直接キック実行
- `set_kicker_charging` (frootspi_msgs/srv/SetKickerCharging): 充電制御

### Service Client （呼び出すサービス）
- ハードウェア制御サービス（キッカー制御用）

## キックタイプ

1. **ストレートキック** (`KICK_TYPE_STRAIGHT`)
   - 地面と平行にボールを蹴る
   - パス、シュート用

2. **チップキック** (`KICK_TYPE_CHIP`)
   - ボールを浮かせて蹴る
   - 障害物回避用

3. **放電** (`KICK_TYPE_DISCHARGE`)
   - 安全のためのコンデンサ放電
   - 電源切断前に実行

## 安全機能

- 充電タイムアウト
- 過電圧保護
- 緊急放電機能
- 連続キック制限

## 使用例

```bash
# キッカー充電開始
ros2 service call /set_kicker_charging frootspi_msgs/srv/SetKickerCharging \
  "{start_charging: true}"

# ストレートキック
ros2 service call /kick frootspi_msgs/srv/Kick \
  "{kick_type: 1, kick_power: 1.0}"

# チップキック
ros2 service call /kick frootspi_msgs/srv/Kick \
  "{kick_type: 2, kick_power: 0.8}"

# 放電
ros2 service call /kick frootspi_msgs/srv/Kick \
  "{kick_type: 3, kick_power: 1.0}"

# 充電停止
ros2 service call /set_kicker_charging frootspi_msgs/srv/SetKickerCharging \
  "{start_charging: false}"
```

## 注意事項

- キック前は必ず充電が必要
- 放電処理は電源切断前に必須
- 連続使用時は過熱に注意