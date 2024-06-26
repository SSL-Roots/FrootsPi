// Copyright 2024 Roots
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

#ifndef FROOTSPI_HARDWARE__KICKER_HPP_
#define FROOTSPI_HARDWARE__KICKER_HPP_

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"

const int GPIO_KICK_STRAIGHT = 7;
const int GPIO_KICK_CHIP = 24;
const int GPIO_KICK_SUPPLY_POWER = 5;
const int GPIO_KICK_ENABLE_CHARGE = 26;
const int GPIO_KICK_CHARGE_COMPLETE = 12;

class Kicker
{
public:
  Kicker();
  ~Kicker();

  bool open(int pi, int pin_ball_sensor);
  bool enableCharging();
  bool disableCharging();
  bool discharge();
  bool kickStraight(uint32_t powerMmps);

  void enableDebugMode();
  void disableDebugMode();
  void debugEnableCharging();
  void debugDisableCharging();

private:
  bool cancelKick();
  bool generateWave(gpioPulse_t * wave, size_t num_pulses);
  bool enableCharging_();
  bool disableCharging_();

  int pi_;
  bool is_charging_;
  bool debug_mode_;
};

#endif  // FROOTSPI_HARDWARE__KICKER_HPP_
