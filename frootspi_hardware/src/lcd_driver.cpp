// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pigpiod_if2.h>

#include <chrono>
#include <string>
#include "frootspi_hardware/lcd_driver.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("lcd_driver");

LCDDriver::LCDDriver()
: pi_(-1)
{
}

LCDDriver::~LCDDriver()
{
}

bool LCDDriver::open(const int pi)
{
  // Ref: http://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_open
  const unsigned BUS = 1;  // I2C1 (GPIO2, 3) を使用
  const unsigned ADDR = 0x3e;  // AQM0802 のスレーブアドレス

  const unsigned FLAGS = 0;

  i2c_handler_ = i2c_open(pi, BUS, ADDR, FLAGS);

  if (i2c_handler_ < 0) {
    return false;
  }

  pi_ = pi;

  if (!init_device()) {
    RCLCPP_ERROR(LOGGER, "Failed to initialzie AQM0802A");
    return false;
  }

  return true;
}

bool LCDDriver::close()
{
  int result = i2c_close(pi_, i2c_handler_);

  if (result >= 0) {
    return true;
  } else {
    return false;
  }
}

bool LCDDriver::write_texts(const std::string text1, const std::string text2)
{
  // LCDの全行に文字列を書き込む関数
  // 文字列を書き込む前に、LCDの表示をクリアする
  // アスキーコードと半角カタカナに対応。それ以外の文字は空白になる
  // 2バイトや4バイト文字を入力されると、正しくLCDに表示できないので注意
  // 文字コードの参考: http://ash.jp/code/unitbl1.htm

  const unsigned START_ADDRESS_FIRST_LINE = 0x00;
  const unsigned START_ADDRESS_SECOND_LINE = 0x40;

  clear_display();
  set_address(START_ADDRESS_FIRST_LINE);
  write_line(text1);
  set_address(START_ADDRESS_SECOND_LINE);
  write_line(text2);

  return true;
}

bool LCDDriver::init_device()
{
  // AQM0802Aの初期設定
  // Ref: http://akizukidenshi.com/download/ds/xiamen/AQM0802.pdf
  bool bus_8bit = true;
  bool display_2line = true;
  bool double_height_font = false;

  unsigned instruction_table = 0;
  if (!set_function(bus_8bit, display_2line, double_height_font, instruction_table)) {
    RCLCPP_ERROR(LOGGER, "Failed to set function");
    return false;
  }

  instruction_table = 1;
  if (!set_function(bus_8bit, display_2line, double_height_font, instruction_table)) {
    RCLCPP_ERROR(LOGGER, "Failed to set instruction table 1");
    return false;
  }

  if (!set_osc_freq(false, true, false, false)) {
    RCLCPP_ERROR(LOGGER, "Failed to set oscillator frequency");
    return false;
  }

  if (!set_contrast(false, true, 32)) {
    RCLCPP_ERROR(LOGGER, "Failed to set contrast");
    return false;
  }

  if (!set_follower_control(true, true, false, false)) {
    RCLCPP_ERROR(LOGGER, "Failed to set follower control");
    return false;
  }

  instruction_table = 0;
  if (!set_function(bus_8bit, display_2line, double_height_font, instruction_table)) {
    RCLCPP_ERROR(LOGGER, "Failed to set instruction table 0");
    return false;
  }

  if (!turn_on_display(true, false, false)) {
    RCLCPP_ERROR(LOGGER, "Failed to turn on display");
    return false;
  }

  if (!clear_display()) {
    RCLCPP_ERROR(LOGGER, "Failed to clear display");
    return false;
  }

  return true;
}

bool LCDDriver::write_command(const unsigned command)
{
  const unsigned CONTROL_BYTE = 0x00;
  int retval = i2c_write_byte_data(pi_, i2c_handler_, CONTROL_BYTE, command);
  if (retval != 0) {
    RCLCPP_ERROR(LOGGER, "Failed to write command. command:%d, retval:%d", command, retval);
    return false;
  }
  // AQM0802の仕様より、26.3 usec以上待機する必要あり
  rclcpp::sleep_for(std::chrono::microseconds(27));
  return true;
}

bool LCDDriver::write_data(const unsigned data)
{
  const unsigned CONTROL_BYTE = 0x40;
  int retval = i2c_write_byte_data(pi_, i2c_handler_, CONTROL_BYTE, data);
  if (retval != 0) {
    RCLCPP_ERROR(LOGGER, "Failed to write data. data:%d, retval:%d", data, retval);
    return false;
  }
  // AQM0802の仕様より、26.3 usec以上待機する必要あり
  rclcpp::sleep_for(std::chrono::microseconds(27));
  return true;
}

bool LCDDriver::set_function(
  const bool bus_8bit, const bool disp_2line,
  const bool double_height_font, const unsigned instruction_table)
{
  unsigned command = 0x20;
  command |= bus_8bit << 4;
  command |= disp_2line << 3;
  command |= double_height_font << 2;
  command |= instruction_table << 0;

  return write_command(command);
}

bool LCDDriver::set_osc_freq(const bool bias, const bool f2, const bool f1, const bool f0)
{
  // Instruction Table を1にしないと設定できない
  // 内部クロックの周波数を決める
  // 詳細はST7032のデータシートの表を見て
  // https://strawberry-linux.com/pub/ST7032i.pdf

  unsigned command = 0x10;
  command |= bias << 3;
  command |= f2 << 2;
  command |= f1 << 1;
  command |= f0 << 0;

  return write_command(command);
}

bool LCDDriver::set_contrast_lowbits(const bool c3, const bool c2, const bool c1, const bool c0)
{
  // Instruction Table を1にしないと設定できない
  // コントラスト(C5 ~ C0)のうちLow Byteをセットする
  // 詳細はST7032のデータシートの表を見て
  // https://strawberry-linux.com/pub/ST7032i.pdf
  unsigned command = 0x70;
  command |= c3 << 3;
  command |= c2 << 2;
  command |= c1 << 1;
  command |= c0 << 0;

  return write_command(command);
}

bool LCDDriver::set_contrast_highbits(
  const bool icon_display_on, const bool booster_on,
  const bool c5, const bool c4)
{
  // Instruction Table を1にしないと設定できない
  // ICONと昇圧回路をON/OFFする
  // コントラスト(C5 ~ C0)のうちC5, C4をセットする
  // 詳細はST7032のデータシートの表を見て
  // https://strawberry-linux.com/pub/ST7032i.pdf
  unsigned command = 0x50;
  command |= icon_display_on << 3;
  command |= booster_on << 2;
  command |= c5 << 1;
  command |= c4 << 0;

  return write_command(command);
}

bool LCDDriver::set_contrast(
  const bool icon_display_on, const bool booster_on,
  const unsigned contrast)
{
  // Instruction Table を1にしないと設定できない
  // ICONと昇圧回路をON/OFFする
  // コントラスト(C5 ~ C0)をセットする 0x00 ~ 0x3F (0 ~ 63)
  // 詳細はST7032のデータシートの表を見て
  // https://strawberry-linux.com/pub/ST7032i.pdf
  unsigned c5 = 0x01 & (contrast >> 5);
  unsigned c4 = 0x01 & (contrast >> 4);
  unsigned c3 = 0x01 & (contrast >> 3);
  unsigned c2 = 0x01 & (contrast >> 2);
  unsigned c1 = 0x01 & (contrast >> 1);
  unsigned c0 = 0x01 & (contrast >> 0);

  unsigned command = 0x70;
  command |= c3 << 3;
  command |= c2 << 2;
  command |= c1 << 1;
  command |= c0 << 0;
  bool retval = write_command(command);

  command = 0x50;
  command |= icon_display_on << 3;
  command |= booster_on << 2;
  command |= c5 << 1;
  command |= c4 << 0;
  return retval && write_command(command);
}

bool LCDDriver::set_follower_control(
  const bool follower_on, const bool rab2,
  const bool rab1, const bool rab0)
{
  // Instruction Table を1にしないと設定できない
  // ボルテージフォロワをON/OFFし、増幅率設定用の抵抗Rab2~0を設定する
  // 詳細はST7032のデータシートの表を見て
  // https://strawberry-linux.com/pub/ST7032i.pdf
  unsigned command = 0x60;
  command |= follower_on << 3;
  command |= rab2 << 2;
  command |= rab1 << 1;
  command |= rab0 << 0;

  bool retval = write_command(command);
  // 電源が安定するまで待機
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  return retval;
}

bool LCDDriver::turn_on_display(
  const bool display_on, const bool cursor_on,
  const bool cursor_blink_on)
{
  // ディスプレイ、カーソルの表示ON/OFF設定
  unsigned command = 0x08;
  command |= display_on << 2;
  command |= cursor_on << 1;
  command |= cursor_blink_on << 0;
  return write_command(command);
}

bool LCDDriver::clear_display()
{
  // ディスプレイ全消去(RAMクリア)
  bool retval = write_command(0x01);
  return retval;
}


bool LCDDriver::set_address(const unsigned address)
{
  // カーソルを移動（RAMのアドレスを変更）
  // アドレスとディスプレイの関係
  // 1行目: 0x00 01 02 03 04 05 06 07
  // 2行目: 0x40 41 42 43 44 45 46 47
  if (address <= 0x07 || (address >= 0x40 && address <= 0x47)) {
    unsigned command = 0x80 | address;
    return write_command(command);
  } else {
    RCLCPP_ERROR(LOGGER, "Invalid LCD RAM address :%x", address);
    return false;
  }
}

void LCDDriver::write_line(const std::string text)
{
  // LCDに文字列を1行書き込む
  // この関数を実行する前に、書き込みアドレスを左端にセットすること

  const int NUM_OF_CHARACTERS = 8;  // LCD 1行の文字数
  const int TEXT_SIZE = text.size();

  int count_characters = 0;
  for (int i = 0; i < TEXT_SIZE; i++) {
    char character = text[i];

    if (character >= 0x20 && character <= 0x7d) {
      // ASCIIの半角英数・記号
      write_data(character);
      count_characters++;
    } else if (character == 0xef && TEXT_SIZE - i >= 3) {
      // 半角カタカナ(3バイト文字)
      char hankaku_katakana = 0x3f;  // ?
      if (text[i + 1] == 0xbd) {
        hankaku_katakana = text[i + 2];
      } else if (text[i + 1] == 0xbe) {
        hankaku_katakana = text[i + 2] + 0x40;
      }
      i += 2;  // 3バイト文字なので、インクリメントしてずらす
      write_data(hankaku_katakana);
      count_characters++;
    }

    if (count_characters >= NUM_OF_CHARACTERS) {
      // 最大文字数を超えた場合は終了
      break;
    }
  }
}
