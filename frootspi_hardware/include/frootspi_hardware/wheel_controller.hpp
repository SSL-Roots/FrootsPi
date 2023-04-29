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

#ifndef FROOTSPI_HARDWARE__WHEEL_CONTROLLER_HPP_
#define FROOTSPI_HARDWARE__WHEEL_CONTROLLER_HPP_

class WheelController
{
public:

  enum ErrorCode
  {
    ERROR_NONE = 0,
    ERROR_GAIN_SETTING_MODE_ENABLED,
    ERROR_GAIN_SETTING_MODE_DISABLED,
    ERROR_CAN_SEND_FAILED,
    ERROR_WHEELS_ARE_MOVING,

  };

  WheelController();
  ~WheelController();

  bool device_open();
  bool device_close();
  ErrorCode set_wheel_velocities(
    const double vel_front_right, const double vel_front_left,
    const double vel_back_center);

  ErrorCode enable_gain_setting();
  ErrorCode disable_gain_setting();
  ErrorCode set_p_gain(const double gain_p);
  ErrorCode set_i_gain(const double gain_i);
  ErrorCode set_d_gain(const double gain_d);
  ErrorCode set_pid_gain(
    const double gain_p, const double gain_i,
    const double gain_d);

private:
  bool send_pid_gain();
  bool send_can(const struct can_frame & frame);
  double constrain(const double value, const double min, const double max);
  bool is_motor_stopping();

  int socket_;
  double vel_front_right_, vel_front_left_, vel_back_center_;
  double gain_p_, gain_i_, gain_d_;
  bool is_gain_setting_enabled_;
};

#endif  // FROOTSPI_HARDWARE__WHEEL_CONTROLLER_HPP_
