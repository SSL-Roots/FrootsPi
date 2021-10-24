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
#include "frootspi_hardware/capacitor_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("capacitor_monitor");
static const float MCP3221_RESOLUTION = 4096;  // 分解能12bit
static const float MCP3221_VDD = 5;  // MCP3203 VDD電圧
static const float CAPACITOR_VOLTAGE_RATIO = 53.18;  // CAPACITOR電圧分圧比　逆数
static const int MAX_THRESH_STATUS_NUM = 4;
static const float CAPACITOR_STATUS_THRESH_VOLTAGE[MAX_THRESH_STATUS_NUM] =
{12.8, 180, 190, 195};

CapacitorMonitor::CapacitorMonitor()
: pi_(-1)
{
}

CapacitorMonitor::~CapacitorMonitor()
{
}

bool CapacitorMonitor::open(const int pi)
{
  // Ref: http://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_open
  const unsigned BUS = 1;  // I2C1 (GPIO2, 3) を使用
  const unsigned ADDR = 0x4d;  // AQM0802 のスレーブアドレス

  const unsigned FLAGS = 0;

  i2c_handler_ = i2c_open(pi, BUS, ADDR, FLAGS);

  if (i2c_handler_ < 0) {
    return false;
  }

  pi_ = pi;

  return true;
}

bool CapacitorMonitor::close()
{
  int result = i2c_close(pi_, i2c_handler_);

  if (result >= 0) {
    return true;
  } else {
    return false;
  }
}

bool CapacitorMonitor::capacitor_info_read(float & voltage, unsigned char & voltage_status)
{
  float read_data = 0;
  unsigned char status_num = 0;

  if (!control_register(&read_data)) {
    RCLCPP_ERROR(LOGGER, "Failed to read from MCP3221 AD .");
    return false;
  }
  voltage = read_data * CAPACITOR_VOLTAGE_RATIO;

  for (int i = MAX_THRESH_STATUS_NUM; i > 0; i--) {
    if (voltage < CAPACITOR_STATUS_THRESH_VOLTAGE[i - 1]) {
      continue;
    } else {
      status_num = i;
      break;
    }
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

bool CapacitorMonitor::control_register(float * read_data)
{
  // Reference: http://ww1.microchip.com/downloads/jp/DeviceDoc/21732D_JP.pdf
  // Page 17.
  char rx_data[2] = {0};

  if (i2c_read_device(pi_, i2c_handler_, rx_data, 2) < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to MCP3221 AD");
    return false;
  }

  *read_data =
    static_cast<float>((rx_data[0] & 0x0f) << 8 | rx_data[1]) / MCP3221_RESOLUTION * MCP3221_VDD;
  return true;
}
