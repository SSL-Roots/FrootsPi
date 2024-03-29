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
#include "frootspi_hardware/battery_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("battery_monitor");
static const char MCP3202_MSB = 1;  // High：Bigエンディアン　Low：Littleエンディアン
static const char MCP3202_CH0 = 0b10;  // Ch0電位取得
static const char MCP3202_CH1 = 0b11;  // Ch1電位取得
static const float MCP3202_RESOLUTION = 4096;  // 分解能12bit
static const float MCP3202_VDD = 3.3;  // MCP3203 VDD電圧
static const float MAIN_BATTERY_VOLTAGE_RATIO = 6.1;  // BATTERY電圧分圧比　逆数
static const float SUB_BATTERY_VOLTAGE_RATIO = 3;  // UPS電圧分圧比 逆数
static const int MAX_THRESH_STATUS_NUM = 4;
static const float MAIN_BATTERY_STATUS_THRESH_VOLTAGE[MAX_THRESH_STATUS_NUM] =
{12.8, 14.8, 15.5, 16.8};
static const float SUB_BATTERY_STATUS_THRESH_VOLTAGE[MAX_THRESH_STATUS_NUM] = {6.5, 6.8, 8, 9};

BatteryMonitor::BatteryMonitor()
: pi_(-1)
{
}

BatteryMonitor::~BatteryMonitor()
{
}

bool BatteryMonitor::open(const int pi)
{
  // Ref: http://abyz.me.uk/rpi/pigpio/pdif2.html#spi_open
  const unsigned CHANNEL = 2;  // Chip select
  const unsigned BAUDRATE = 900000;  // MCP3202の最大クロック周波数は0.9MHz
  const unsigned FLAG_MODE = 0b00;  // SPIモード
  const unsigned FLAG_PX = 0b000;  // CE0~2の論理設定、0でアクティブロー
  const unsigned FLAG_UX = 0b000;  // CE0~2のGPIO設定、0でGPIOをSPI用に確保
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

  auto message = "SPI FLAGS:" + std::bitset<22>(FLAGS).to_string();
  RCLCPP_DEBUG(LOGGER, message.c_str());

  spi_handler_ = spi_open(pi, CHANNEL, BAUDRATE, FLAGS);

  if (spi_handler_ < 0) {
    return false;
  }

  pi_ = pi;

  return true;
}

bool BatteryMonitor::close()
{
  int result = spi_close(pi_, spi_handler_);

  if (result >= 0) {
    return true;
  } else {
    return false;
  }
}

bool BatteryMonitor::main_battery_info_read(float & voltage, unsigned char & voltage_status)
{
  float read_data = 0;
  unsigned char status_num = 0;

  if (!read_adc(MCP3202_CH0, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to read from AD CH0.");
    return false;
  }
  voltage = read_data * MAIN_BATTERY_VOLTAGE_RATIO;

  for (int i = MAX_THRESH_STATUS_NUM; i > 0; i--) {
    if (voltage < MAIN_BATTERY_STATUS_THRESH_VOLTAGE[i - 1]) {
      continue;
    }
    status_num = i;
    break;
  }

  switch (status_num) {
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_UNKNOWN:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_UNKNOWN;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_NEED_CHARGING:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_NEED_CHARGING;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_OK:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_OK;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_FULL:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_FULL;
      break;
    default:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_UNKNOWN;
      break;
  }

  return true;
}

bool BatteryMonitor::sub_battery_info_read(float & voltage, unsigned char & voltage_status)
{
  float read_data = 0;
  unsigned char status_num = 0;

  if (!read_adc(MCP3202_CH1, &read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to read from AD CH1.");
    return false;
  }
  voltage = read_data * SUB_BATTERY_VOLTAGE_RATIO;

  for (int i = MAX_THRESH_STATUS_NUM; i > 0; i--) {
    if (voltage < SUB_BATTERY_STATUS_THRESH_VOLTAGE[i - 1]) {
      continue;
    }
    status_num = i;
    break;
  }

  switch (status_num) {
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_UNKNOWN:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_UNKNOWN;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_NEED_CHARGING:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_NEED_CHARGING;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_OK:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_OK;
      break;
    case frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_FULL:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_FULL;
      break;
    default:
      voltage_status = frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_UNKNOWN;
      break;
  }

  return true;
}

bool BatteryMonitor::read_adc(const char channel, float * read_data)
{
  // Reference: http://ww1.microchip.com/downloads/jp/DeviceDoc/21034A_JP.pdf
  // Page 6.
  char tx_data[3] = {0};
  char rx_data[3] = {0};
  // op code = 0b 0001 {SGL}{ODD}{MSBF}0 0000 0000 0000
  tx_data[0] = 0x01;
  tx_data[1] |= channel << 6;
  tx_data[1] |= MCP3202_MSB << 5;
  tx_data[2] = 0x00;

  if (spi_xfer(pi_, spi_handler_, tx_data, rx_data, 3) < 0) {
    auto message = "Failed to AD channel:" + std::to_string(channel);
    RCLCPP_ERROR(LOGGER, message.c_str());
    return false;
  }

  *read_data =
    static_cast<float>((rx_data[1] & 0x0f) << 8 | rx_data[2]) / MCP3202_RESOLUTION * MCP3202_VDD;
  return true;
}
