# frootspi_examples モジュール

## 概要

`frootspi_examples`は、FrootsPiシステムの起動と設定例を提供するモジュールです。各種launchファイル、設定ファイル、テストスクリプトが含まれています。

## 主な機能

- システム起動設定（launchファイル）
- 設定例とテンプレート
- ハードウェアテストスクリプト
- systemd自動起動設定

## 含まれるファイル

### Launchファイル
- `robot.launch.py`: 完全なロボットシステム起動
- `conductor.launch.py`: 制御部のみ起動
- `hardware.launch.py`: ハードウェア部のみ起動  
- `joycon.launch.py`: ジョイスティック制御起動

### テストスクリプト
- `hardware_test.sh`: ハードウェア動作テスト
- 個別モジュールテスト用スクリプト

### 設定ファイル
- ロボット設定テンプレート
- GPIO設定例
- コントローラー設定例

## 起動方法

### 完全システム起動
```bash
ros2 launch frootspi_examples robot.launch.py
```

### ジョイスティック制御
```bash
ros2 launch frootspi_examples joycon.launch.py
```

### ハードウェアテスト
```bash
# 各ハードウェアの動作確認
bash frootspi_examples/hardware_test.sh
```

## 自動起動設定

systemdサービスとしてロボットを自動起動：

```bash
# 自動起動設定のインストール
./frootspi_examples/systemd/register_systemd.sh

# サービス確認
systemctl status frootspi-robot
```

## 設定ファイルの場所

- `~/robot_config.yaml`: メイン設定ファイル
- `config/gpio.yaml`: GPIO設定
- `config/joy_config.yaml`: コントローラー設定

## デバッグ用コマンド

```bash
# CAN通信の開始
sudo ip link set can0 up type can bitrate 1000000

# 個別トピック確認
ros2 topic list
ros2 topic echo /target_velocity
ros2 topic echo /ball_detection

# サービス一覧
ros2 service list
```

## カスタマイズ

- launchファイルの編集で起動モジュール選択
- 設定ファイルでハードウェア設定変更
- systemd設定で自動起動カスタマイズ