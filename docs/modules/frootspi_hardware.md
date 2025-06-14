# frootspi_hardware モジュール

## 概要

`frootspi_hardware`は、FrootsPiの物理ハードウェアとの直接インターフェースを担当するモジュールです。GPIO、センサー、LED、スイッチなどの低レベルハードウェア制御を行います。

## 主な機能

- GPIO制御（pigpioライブラリ使用）
- センサーデータ読み取り（ボールセンサー等）
- LED制御
- スイッチ状態監視
- ハードウェア設定管理

## インターフェース

### Publication （送信トピック）
- `ball_detection` (frootspi_msgs/BallDetection): ボール検出情報
- `switch_state` (std_msgs/Bool): スイッチ状態

### Service Server （提供するサービス）
- `set_left_led` (std_srvs/SetBool): 左LED制御
- `set_center_led` (std_srvs/SetBool): 中央LED制御  
- `set_right_led` (std_srvs/SetBool): 右LED制御
- `enable_gain_setting` (std_srvs/SetBool): ゲイン設定モード制御

### パラメータ
- GPIO設定（gpio.yamlから読み込み）
- センサー閾値設定
- サンプリングレート設定

## ハードウェア構成

### センサー
- ボールセンサー: ボールの近接検出
- 各種スイッチ: メカニカル動作確認用

### アクチュエータ
- LED (3個): 状態表示用
- その他GPIO制御機器

## 使用例

```bash
# LED制御例
ros2 service call /set_center_led std_srvs/srv/SetBool "{data: true}"

# ボール検出状態確認
ros2 topic echo /ball_detection

# スイッチ状態確認  
ros2 topic echo /switch_state
```

## 設定ファイル

- `config/gpio.yaml`: GPIO設定ファイル