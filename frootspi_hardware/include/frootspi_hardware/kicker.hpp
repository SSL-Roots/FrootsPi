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

#pragma once

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
    void enableCharging();
    void disableCharging();
    bool discharge();
    bool kickStraight(uint32_t powerMmps);

private:
    bool cancelKick();

    rclcpp::TimerBase::SharedPtr discharge_kicker_timer_;

    int pi_;
    bool is_charging_;
};