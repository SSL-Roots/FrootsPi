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
  WheelController();
  ~WheelController();

  bool device_open();
  bool device_close();
  bool set_wheel_velocities(
    const double vel_front_right, const double vel_front_left,
    const double vel_back_center);

private:
  int socket_;
};

#endif  // FROOTSPI_HARDWARE__WHEEL_CONTROLLER_HPP_
