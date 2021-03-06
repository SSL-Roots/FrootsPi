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

#ifndef FROOTSPI_HARDWARE__BATTERY_MONITOR_HPP_
#define FROOTSPI_HARDWARE__BATTERY_MONITOR_HPP_

#include <stdio.h>
#include "frootspi_msgs/msg/battery_voltage.hpp"

class BatteryMonitor
{
public:
  BatteryMonitor();
  ~BatteryMonitor();

  bool open(const int pi);
  bool close();
  bool main_battery_info_read(float & voltage, unsigned char & voltage_status);
  bool sub_battery_info_read(float & voltage, unsigned char & voltage_status);

private:
  bool read_adc(const char channel, float * read_data);
  int spi_handler_;
  int pi_;
};

#endif  // FROOTSPI_HARDWARE__BATTERY_MONITOR_HPP_
