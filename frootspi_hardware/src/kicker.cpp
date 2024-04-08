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

#include "frootspi_hardware/kicker.hpp"

/**
 * @brief コンストラクタ。キッカーの初期化を行います。
 */
Kicker::Kicker()
: is_charging_(false)
{
}

/**
 * @brief デストラクタ。キッカーのリソースを解放します。
 */
Kicker::~Kicker()
{
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);
}

/**
 * @brief キッカーを開く。
 *
 * @param pi ライブラリのピン番号。
 * @param pin_ball_sensor ボールセンサーのピン番号。
 * @return true キッカーの開始に成功した場合。
 * @return false キッカーの開始に失敗した場合。
 */
bool Kicker::open(int pi, int pin_ball_sensor)
{
  this->pi_ = pi;

  // ボールセンサーの設定
  set_mode(pi_, pin_ball_sensor, PI_INPUT);
  set_pull_up_down(pi_, pin_ball_sensor, PI_PUD_UP);

  // キッカーの設定
  set_mode(pi_, GPIO_KICK_STRAIGHT, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_LOW);
  set_mode(pi_, GPIO_KICK_CHIP, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_CHIP, PI_LOW);
  set_mode(pi_, GPIO_KICK_SUPPLY_POWER, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);
  set_mode(pi_, GPIO_KICK_ENABLE_CHARGE, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  set_mode(pi_, GPIO_KICK_CHARGE_COMPLETE, PI_INPUT);
  set_pull_up_down(pi_, GPIO_KICK_CHARGE_COMPLETE, PI_PUD_UP);    // 外部プルアップあり
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_HIGH);

  return true;
}

/**
 * @brief ストレートキックを実行します。
 *
 * @param powerMmps キックの強さ（ミリメートル毎秒）。
 * @return true キックの実行に成功した場合。
 * @return false キックの実行に失敗した場合。
 */
bool Kicker::kickStraight(uint32_t powerMmps)
{
  const int MAX_SLEEP_TIME_USEC_FOR_STRAIGHT = 30000;

  if (this->debug_mode_) {return false;}

  uint32_t sleep_time_usec = 4 * powerMmps + 100;  // 実験に基づく定数
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

  bool gen_result = this->generateWave(pulses, sizeof(pulses) / sizeof(gpioPulse_t));
  if (!gen_result) {
    return false;
  }

  return true;
}

/**
 * @brief 充電を有効にします。
 * @return true 充電許可に成功した場合。
 * @return false 充電許可に失敗した場合。
 */
bool Kicker::enableCharging()
{
  if (this->debug_mode_) {return false;}
  return this->enableCharging_();
}

/**
 * @brief 充電を無効にします。
 * @return true 充電禁止に成功した場合。
 * @return false 充電禁止に失敗した場合。
 */
bool Kicker::disableCharging()
{
  if (this->debug_mode_) {return false;}
  return this->disableCharging_();
}

/**
 * @brief 放電を行います。
 *
 * @return true 放電に成功した場合。
 * @return false 放電に失敗した場合。
 */
bool Kicker::discharge()
{
  uint32_t bit_kick_straight = 1 << GPIO_KICK_STRAIGHT;
  uint32_t bit_kick_enable_charge = 1 << GPIO_KICK_ENABLE_CHARGE;

  const uint32_t ontime_us = 1000;
  const uint32_t cycle_us = 500 * 1000;
  const size_t num_discharge = 60;
  const size_t kPulseArrayLength = num_discharge * 2 + 1;

  gpioPulse_t pulses[kPulseArrayLength];
  pulses[0] = {0, bit_kick_enable_charge, 100};     // 充電をOFF
  for (size_t i = 0; i < num_discharge; i++) {
    pulses[i * 2 + 1] = {bit_kick_straight, 0, ontime_us};
    pulses[i * 2 + 2] = {0, bit_kick_straight, cycle_us - ontime_us};
  }

  bool gen_result = this->generateWave(pulses, kPulseArrayLength);
  if (!gen_result) {
    return false;
  }

  this->is_charging_ = false;
  return true;
}

/**
 * @brief デバッグモードを有効にします。
 */
void Kicker::enableDebugMode()
{
  this->debug_mode_ = true;
}

/**
 * @brief デバッグモードを無効にします。
 */
void Kicker::disableDebugMode()
{
  this->debug_mode_ = false;
}

/**
 * @brief デバッグモードで充電を有効にします。
 * @return true 充電許可に成功した場合。
 * @return false 充電許可に失敗した場合。
 */
void Kicker::debugEnableCharging()
{
  this->enableCharging_();
}

/**
 * @brief デバッグモードで充電を無効にします。
 * @return true 充電禁止に成功した場合。
 * @return false 充電禁止に失敗した場合。
 */
void Kicker::debugDisableCharging()
{
  this->disableCharging_();
}


/**
 * @brief キックをキャンセルします。
 *
 * @return true キックのキャンセルに成功した場合。
 * @return false キックのキャンセルに失敗した場合。
 */
bool Kicker::cancelKick()
{
  uint32_t bit_kick_straight = 1 << GPIO_KICK_STRAIGHT;

  gpioPulse_t pulses[] = {
    {0, bit_kick_straight, 1},   // キックOFF
  };

  return this->generateWave(pulses, sizeof(pulses) / sizeof(gpioPulse_t));
}

/**
 * @brief 波形を生成します。
 *
 * @param wave 波形の配列。
 * @param num_pulses 波形の数。
 * @return true 波形の生成に成功した場合。
 * @return false 波形の生成に失敗した場合。
 */
bool Kicker::generateWave(gpioPulse_t * wave, size_t num_pulses)
{
  wave_clear(pi_);
  int num_pulse = wave_add_generic(pi_, num_pulses, wave);
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

  wave_delete(pi_, wave_id);    // wave の数が無限に増えないよう逐次削除
                                // waveのバッファから消えるだけで、波形自体は出る
  return true;
}

/**
 * @brief 充電を有効にします。
 * @return true 充電許可に成功した場合。
 * @return false 充電許可に失敗した場合。
 */
bool Kicker::enableCharging_()
{
  if (this->is_charging_) {return false;}

  this->cancelKick();
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_HIGH);
  this->is_charging_ = true;

  return true;
}

/**
 * @brief 充電を無効にします。
 * @return true 充電禁止に成功した場合。
 * @return false 充電禁止に失敗した場合。
 */
bool Kicker::disableCharging_()
{
  if (!this->is_charging_) {return false;}

  this->cancelKick();
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  this->is_charging_ = false;

  return true;
}
