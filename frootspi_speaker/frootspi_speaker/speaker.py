#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2023 Roots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
from frootspi_msgs.msg import SpeakerVoice
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import simpleaudio


# 多重再生を制限するためのバッファクラス
class PlayingBuffer:
    def __init__(self, buffer_size=3):
        self._buffer = [None] * buffer_size

    def _get_empty_index(self):
        for index in range(len(self._buffer)):
            if self._buffer[index] is None:
                return index

            if not self._buffer[index].is_playing():
                return index
        return -1

    def play(self, file_path):
        buffer_index = self._get_empty_index()
        if buffer_index < 0:
            return

        wav_obj = simpleaudio.WaveObject.from_wave_file(file_path)
        self._buffer[buffer_index] = wav_obj.play()

class Speaker(Node):

    def __init__(self):
        super().__init__('speaker')

        self._voices_path = self.declare_parameter('voices_path', "").value
        self._file_list_path = self._voices_path + "/" + \
            self.declare_parameter('file_list_name', "voice_file_list.csv").value

        # TODO(ShotaAk): CSVファイルのフォーマットもチェックしたい
        if not os.path.isfile(self._file_list_path):
            self.get_logger().info('file exist')
            raise ValueError("File:{} does not exist!".format(self._file_list_path))

        self._file_dict = self.make_file_dict(self._file_list_path)

        self._playing_buffer = PlayingBuffer(buffer_size=1)
        self._sub = self.create_subscription(SpeakerVoice, 'speaker_voice', self.callback, 10)

    def make_file_dict(self, file_list_path):
        # SpeakerVoice定数（数値）をキーとした辞書を作成する
        file_dict = {}
        with open(file_list_path, 'r') as f:
            reader = csv.DictReader(f)
            for r in reader:
                voice_type = r["type"]
                if hasattr(SpeakerVoice, voice_type):
                    file_dict[getattr(SpeakerVoice, voice_type)] = r["file_name"]

        return file_dict

    def callback(self, msg):
        if not msg.voice_type in self._file_dict:
            self.get_logger().info('Voice type:{} does not defined.'.format(msg.voice_type))
            return

        voice_file_path = self._voices_path + "/" + self._file_dict[msg.voice_type]
        if not os.path.isfile(voice_file_path) or not voice_file_path.endswith(".wav"):
            self.get_logger().info('File:{} is not wav file.'.format(voice_file_path))
            return

        self._playing_buffer.play(voice_file_path)


def main(args=None):
    rclpy.init(args=args)

    node = Speaker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, ValueError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
