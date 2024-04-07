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

#ifndef FROOTSPI_HARDWARE__DRIBBLER_HPP_
#define FROOTSPI_HARDWARE__DRIBBLER_HPP_

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"

static const int GPIO_DRIBBLE_PWM = 13;
static const int DRIBBLE_PWM_FREQUENCY = 40000;  // kHz
static const int DRIBBLE_PWM_DUTY_CYCLE = 1e6 / DRIBBLE_PWM_FREQUENCY;  // usec

class Dribbler
{
public:
  Dribbler();
  ~Dribbler();

  bool open(int pi);
  void close();

  bool drive(double power);

private:
  int pi_;
};

#endif  // FROOTSPI_HARDWARE__DRIBBLER_HPP_
