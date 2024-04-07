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

#include <pigpiod_if2.h>
#include <chrono>

#include "frootspi_hardware/kicker.hpp"


Kicker::Kicker() :
    is_charging_(false)
{
}

Kicker::~Kicker()
{
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);
}

bool Kicker::open(int pi, int pin_ball_sensor)
{
    this->pi_ = pi;

    // ball sensor setup
    set_mode(pi_, pin_ball_sensor, PI_INPUT);
    set_pull_up_down(pi_, pin_ball_sensor, PI_PUD_UP);

    // kicker setup
    set_mode(pi_, GPIO_KICK_STRAIGHT, PI_OUTPUT);
    gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_LOW);
    set_mode(pi_, GPIO_KICK_CHIP, PI_OUTPUT);
    gpio_write(pi_, GPIO_KICK_CHIP, PI_LOW);
    set_mode(pi_, GPIO_KICK_SUPPLY_POWER, PI_OUTPUT);
    gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);
    set_mode(pi_, GPIO_KICK_ENABLE_CHARGE, PI_OUTPUT);
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
    set_mode(pi_, GPIO_KICK_CHARGE_COMPLETE, PI_INPUT);
    set_pull_up_down(pi_, GPIO_KICK_CHARGE_COMPLETE, PI_PUD_UP);  // 外部プルアップあり
    gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_HIGH);

    return true;
}


bool Kicker::kickStraight(uint32_t powerMmps)
{
  const int MAX_SLEEP_TIME_USEC_FOR_STRAIGHT = 30000;
    // ストレートキック

    uint32_t sleep_time_usec = 4 * powerMmps + 100;  // constants based on test
    if (sleep_time_usec > MAX_SLEEP_TIME_USEC_FOR_STRAIGHT) {
      sleep_time_usec = MAX_SLEEP_TIME_USEC_FOR_STRAIGHT;
    }

    // GPIOをHIGHにしている時間を変化させて、キックパワーを変更する
    uint32_t bit_kick_straight = 1 << GPIO_KICK_STRAIGHT;
    uint32_t bit_kick_enable_charge = 1 << GPIO_KICK_ENABLE_CHARGE;
    uint32_t bit_kick_enable_charge_masked = this->is_charging_ ? bit_kick_enable_charge : 0;

    gpioPulse_t pulses[] = {
      {0, bit_kick_enable_charge, 100},                                           // 充電を停止する
      {bit_kick_straight, 0, sleep_time_usec},                                    // キックON
      {0, bit_kick_straight, 100},                                                // キックOFF
      {bit_kick_enable_charge_masked, 0, 0},                                      // 充電を再開する
    };

    wave_clear(pi_);
    int num_pulse = wave_add_generic(pi_, sizeof(pulses) / sizeof(gpioPulse_t), pulses);
    if (num_pulse < 0) {
        return false;
    }

    int wave_id = wave_create(pi_);
    if (wave_id < 0) {
        return false;
    }

    int result = wave_send_once(pi_, wave_id);
    if (result < 0) {
        return false;
    }

    return true;
}

void Kicker:: enableCharging()
{
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_HIGH);
    this->is_charging_ = true;
}


void Kicker:: disableCharging()
{
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
    this->is_charging_ = false;
}


bool Kicker::discharge()
{
    uint32_t bit_kick_straight = 1 << GPIO_KICK_STRAIGHT;
    uint32_t bit_kick_enable_charge = 1 << GPIO_KICK_ENABLE_CHARGE;

    uint32_t ontime_us = 1000;
    uint32_t cycle_us = 1000 * 1000;

    gpioPulse_t pulses[] = {
      {0, bit_kick_enable_charge, 100},                                           // 充電を停止する
      {bit_kick_straight, 0, ontime_us},
      {0, bit_kick_straight, cycle_us - ontime_us},
      {bit_kick_straight, 0, ontime_us},
      {0, bit_kick_straight, cycle_us - ontime_us},
      {bit_kick_straight, 0, ontime_us},
      {0, bit_kick_straight, cycle_us - ontime_us},
      {bit_kick_straight, 0, ontime_us},
      {0, bit_kick_straight, cycle_us - ontime_us},
      {bit_kick_straight, 0, ontime_us},
      {0, bit_kick_straight, cycle_us - ontime_us},
      {bit_kick_straight, 0, ontime_us},
      {0, bit_kick_straight, cycle_us - ontime_us},
    };

    wave_clear(pi_);
    int num_pulse = wave_add_generic(pi_, sizeof(pulses) / sizeof(gpioPulse_t), pulses);
    if (num_pulse < 0) {
        return false;
    }

    int wave_id = wave_create(pi_);
    if (wave_id < 0) {
        return false;
    }

    int result = wave_send_once(pi_, wave_id);
    if (result < 0) {
        return false;
    }

    return true;
}