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

#include "frootspi_hardware/dribbler.hpp"

/**
 * @brief コンストラクタ。ドリブラーの初期化を行います。
 */
Dribbler::Dribbler()
{
}

/**
 * @brief デストラクタ。ドリブラーのリソースを解放します。
 */
Dribbler::~Dribbler()
{
}

/**
 * @brief ドリブラーを開く。
 *
 * @param pi ライブラリのピン番号。
 */
bool Dribbler::open(int pi)
{
  this->pi_ = pi;

  // dribbler setup
  set_mode(pi_, GPIO_DRIBBLE_PWM, PI_OUTPUT);
  set_PWM_frequency(pi_, GPIO_DRIBBLE_PWM, DRIBBLE_PWM_FREQUENCY);
  set_PWM_range(pi_, GPIO_DRIBBLE_PWM, DRIBBLE_PWM_DUTY_CYCLE);
  gpio_write(pi_, GPIO_DRIBBLE_PWM, PI_HIGH);  // 負論理のためHighでモータオフ

  return true;
}

/**
 * @brief ドリブラーを閉じる。
 */
void Dribbler::close()
{
  gpio_write(pi_, GPIO_DRIBBLE_PWM, PI_HIGH);  // 負論理のためHighでモータオフ
}

/**
 * @brief ドリブラーを駆動する。
 *
 * @param power ドリブラーの駆動力。0.0で停止、1.0で最大出力。
 */
bool Dribbler::drive(double power)
{
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 0.0) {
    power = 0.0;
  }

  // 負論理のため反転
  // 少数を切り上げるため0.1を足す (例：20.0 -> 19となるので、20.1 -> 20とさせる)
  int dribble_duty_cycle = DRIBBLE_PWM_DUTY_CYCLE * (1.0 - power) + 0.1;

  set_PWM_dutycycle(pi_, GPIO_DRIBBLE_PWM, dribble_duty_cycle);

  return true;
}
