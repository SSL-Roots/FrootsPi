# frootspi_speaker モジュール

## 概要

`frootspi_speaker`は、FrootsPiの音声出力機能を担当するモジュールです。システム状態の音声通知、デバッグ用の音声フィードバックを提供します。

## 主な機能

- 音声ファイル再生
- システム状態の音声通知
- デバッグ用音声フィードバック
- 音量制御

## インターフェース

### Subscription （受信トピック）
- `play_sound` (frootspi_msgs/PlaySound): 音声再生指令
- システム状態トピック（自動音声通知用）

### Service Server （提供するサービス）
- `play_audio` (frootspi_msgs/srv/PlayAudio): 音声ファイル再生
- `set_volume` (frootspi_msgs/srv/SetVolume): 音量設定

## 音声ファイル

システムで使用される音声ファイル：
- 起動完了音
- エラー通知音
- キック完了音
- 充電完了音
- シャットダウン音

## 依存関係

- Python 3
- simpleaudio ライブラリ
- ALSA （Linuxオーディオシステム）

## 使用例

```bash
# 音声再生
ros2 service call /play_audio frootspi_msgs/srv/PlayAudio \
  "{audio_file: 'startup.wav'}"

# 音量設定
ros2 service call /set_volume frootspi_msgs/srv/SetVolume \
  "{volume: 0.7}"

# 音声再生トピック使用
ros2 topic pub /play_sound frootspi_msgs/msg/PlaySound \
  "{sound_name: 'error', repeat: 1}"
```

## 設定

- 音声ファイルパス設定
- デフォルト音量設定
- 音声通知の有効/無効設定

## インストール要件

```bash
# 必要なパッケージのインストール
sudo apt install -y python3-pip libasound2-dev alsa-utils
pip3 install simpleaudio
```