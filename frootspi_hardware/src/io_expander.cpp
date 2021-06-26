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

#include <bitset>
#include "frootspi_hardware/io_expander.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("io_expander");
static const char MCP23S08_READ = 1;
static const char MCP23S08_WRITE = 0;
static const char MCP23S08_REG_IODIR = 0x00;    // 入出力設定
static const char MCP23S08_REG_GPIO = 0x09;     // GPIO
static const char MCP23S08_PIN_A0 = 0;  // A0ピンの値（電位）
static const char MCP23S08_PIN_A1 = 0;  // A1ピンの値（電位）
static const char MCP23S08_GPIO_LED = 0;
static const char MCP23S08_GPIO_PUSHSW0 = 1;
static const char MCP23S08_GPIO_PUSHSW1 = 2;
static const char MCP23S08_GPIO_PUSHSW2 = 3;
static const char MCP23S08_GPIO_PUSHSW3 = 4;
static const char MCP23S08_GPIO_DIPSW0 = 6;
static const char MCP23S08_GPIO_DIPSW1 = 5;

IOExpander::IOExpander()
: pi_(-1)
{
}

IOExpander::~IOExpander()
{
}

bool IOExpander::open(const int pi)
{
  // Ref: http://abyz.me.uk/rpi/pigpio/pdif2.html#spi_open
  const unsigned CHANNEL = 0;  // Chip select
  const unsigned BAUDRATE = 1000000;  // MCP23S08の最大クロック周波数は10MHz
  const unsigned FLAG_MODE = 0b00;  // SPIモード
  const unsigned FLAG_PX = 0b000;  // CE0~2の論理設定、0でアクティブロー
  const unsigned FLAG_UX = 0b110;  // CE0~2のGPIO設定、0でGPIOをSPI用に確保
  const unsigned FLAG_A = 0b1;  // 0でメイン（SPI0）、1でAuxiliary（SPI1）
  const unsigned FLAG_W = 0b0;  // メインSPI専用。1で3線SPIモード
  const unsigned FLAG_N = 0b0000;  // 3線モードでMISO切替時に書き込むバイト数
  const unsigned FLAG_T = 0b0;  // Auxiliary専用。1で最下位ビットを最初に送信する
  const unsigned FLAG_R = 0b0;  // Auxiliary専用。1で最下位ビットを最初に受信する
  const unsigned FLAG_B = 0b000000;  // Auxiliary専用。ワードサイズを設定
  const unsigned FLAGS = 0 | (FLAG_MODE << 0) |
    (FLAG_PX << 2) |
    (FLAG_UX << 5) |
    (FLAG_A << 8) |
    (FLAG_W << 9) |
    (FLAG_N << 10) |
    (FLAG_T << 14) |
    (FLAG_R << 15) |
    (FLAG_B << 16);

  RCLCPP_DEBUG(LOGGER, "SPI FLAGS:" + std::bitset<22>(FLAGS).to_string());

  spi_handler_ = spi_open(pi, CHANNEL, BAUDRATE, FLAGS);

  if (spi_handler_ < 0) {
    return false;
  }

  pi_ = pi;

  // 入出力ピンの設定
  char write_data = 0xFF ^ (1 << MCP23S08_GPIO_LED);
  char read_data = 0;
  if (!control_register(MCP23S08_REG_IODIR, MCP23S08_WRITE, write_data, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize IODIR register");
    return false;
  }

  if (!control_register(MCP23S08_REG_IODIR, MCP23S08_READ, write_data, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to read IODIR register");
    return false;
  }

  if (read_data != write_data) {
    RCLCPP_ERROR(
      LOGGER, "Failed to set IODIR register to " +
      std::bitset<8>(write_data).to_string() +
      ", actual:" + std::bitset<8>(read_data).to_string());
    return false;
  }

  return true;
}

bool IOExpander::close()
{
  int result = spi_close(pi_, spi_handler_);

  if (result >= 0) {
    return true;
  } else {
    return false;
  }
}

bool IOExpander::read(
  bool & pushed_button0, bool & pushed_button1, bool & pushed_button2,
  bool & pushed_button3, bool & turned_on_dip0, bool & turned_on_dip1)
{
  char write_data = 0;
  char read_data = 0;

  if (!control_register(MCP23S08_REG_GPIO, MCP23S08_READ, write_data, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to read from GPIO.");
    return false;
  }

  // 回路が負論理なので、GPIOのビットをとりだしたあとXORで反転させる
  pushed_button0 = ((read_data >> MCP23S08_GPIO_PUSHSW0) & 1) ^ 1;
  pushed_button1 = ((read_data >> MCP23S08_GPIO_PUSHSW1) & 1) ^ 1;
  pushed_button2 = ((read_data >> MCP23S08_GPIO_PUSHSW2) & 1) ^ 1;
  pushed_button3 = ((read_data >> MCP23S08_GPIO_PUSHSW3) & 1) ^ 1;
  turned_on_dip0 = ((read_data >> MCP23S08_GPIO_DIPSW0) & 1) ^ 1;
  turned_on_dip1 = ((read_data >> MCP23S08_GPIO_DIPSW1) & 1) ^ 1;
  return true;
}

bool IOExpander::write(const bool set_led)
{
  char write_data = 0;
  char read_data = 0;

  // 現在のGPIOの状態を取得
  if (!control_register(MCP23S08_REG_GPIO, MCP23S08_READ, write_data, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to read from GPIO.");
    return false;
  }

  // 特定のビットだけを変更する
  if (set_led) {
    write_data = read_data | (1 << MCP23S08_GPIO_LED);
  } else {
    write_data = read_data & ~(1 << MCP23S08_GPIO_LED);
  }

  if (!control_register(MCP23S08_REG_GPIO, MCP23S08_WRITE, write_data, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to write to GPIO.");
    return false;
  }

  return true;
}

bool IOExpander::control_register(
  const char addr, const char rw, const char write_data,
  char * read_data)
{
  // Reference: http://ww1.microchip.com/downloads/cn/DeviceDoc/cn026496.pdf
  // Page 6.
  char tx_data[3] = {0};
  char rx_data[3] = {0};
  // op code = 0b 0100 0{A1}{A0}{R/W}
  tx_data[0] = 0x40;
  tx_data[0] |= MCP23S08_PIN_A1 << 2;
  tx_data[0] |= MCP23S08_PIN_A0 << 1;
  tx_data[0] |= rw << 0;
  tx_data[1] = addr;
  tx_data[2] = write_data;

  if (spi_xfer(pi_, spi_handler_, tx_data, rx_data, 3) < 0) {
    RCLCPP_ERROR(
      LOGGER, "Failed to transfer addr:" + std::to_string(
        addr) + ", write_data:" + std::to_string(write_data));
    return false;
  }

  *read_data = rx_data[2];
  return true;
}
