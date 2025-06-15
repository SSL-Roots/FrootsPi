# frootspi_joycon モジュール

## 概要

`frootspi_joycon`は、ジョイスティック（ゲームコントローラー）からの入力を受信し、FrootsPi用のRobotCommandに変換するモジュールです。

## 主な機能

- ジョイスティック入力の受信
- ボタン・スティック入力のマッピング
- RobotCommandメッセージの生成
- 複数コントローラー対応

## インターフェース

### Subscription （受信トピック）
- `joy` (sensor_msgs/Joy): ジョイスティック入力

### Publication （送信トピック）
- `command` (consai_frootspi_msgs/RobotCommand): 変換されたロボットコマンド

## サポートコントローラー

設定ファイルにより以下のコントローラーに対応：
- PlayStation コントローラー
- Xbox コントローラー
- 汎用USBゲームパッド

## ボタンマッピング

標準的なマッピング（設定により変更可能）：
- 左スティック: 移動（x, y方向）
- 右スティック: 回転
- R1/RB: キック
- L1/LB: ドリブラー
- △/Y: チップキック切り替え

## 設定ファイル

```yaml
# joy_config例
controller_type: "ps4"
axis_mappings:
  linear_x: 1
  linear_y: 0
  angular_z: 3
button_mappings:
  kick: 5
  dribbler: 4
  chip_kick: 3
```

## 使用例

```bash
# ジョイスティック接続確認
ros2 topic echo /joy

# コントローラー設定でシステム起動
ros2 launch frootspi_examples joycon.launch.py

# 生成されるコマンド確認
ros2 topic echo /command
```

## デバッグ

```bash
# ジョイスティックデバイス確認
ls /dev/input/js*

# joy_nodeの動作確認
ros2 run joy joy_node

# 入力値の確認
ros2 topic echo /joy
```

## 対応入力

- アナログスティック入力
- デジタルボタン入力
- トリガー入力（アナログ）
- 十字キー入力