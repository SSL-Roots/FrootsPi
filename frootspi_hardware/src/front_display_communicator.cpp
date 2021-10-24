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

#include <chrono>
#include <string>
#include "frootspi_hardware/front_display_communicator.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("front_display_communicator");

FrontDisplayCommunicator::FrontDisplayCommunicator()
: pi_(-1)
{
}

FrontDisplayCommunicator::~FrontDisplayCommunicator()
{
}

bool FrontDisplayCommunicator::open(const int pi)
{
  // Ref: http://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_open
  const unsigned BUS = 1;  // I2C1 (GPIO2, 3) を使用
  const unsigned ADDR = 0x60;  // AQM0802 のスレーブアドレス

  const unsigned FLAGS = 0;

  i2c_handler_ = i2c_open(pi, BUS, ADDR, FLAGS);

  if (i2c_handler_ < 0) {
    return false;
  }

  pi_ = pi;

  return true;
}

bool FrontDisplayCommunicator::close()
{
  int result = i2c_close(pi_, i2c_handler_);

  if (result >= 0) {
    return true;
  } else {
    return false;
  }
  
}

bool FrontDisplayCommunicator::send_data(void)
{
  char data[1] = {0};

  front_indicate_data_.Parameter.RobotID = 10;
  front_indicate_data_.Parameter.BatVol = 0*10;
  front_indicate_data_.Parameter.CapVol = 00;
  front_indicate_data_.Parameter.BallSens = false;
  front_indicate_data_.Parameter.CapacitorSta = false;
  front_indicate_data_.Parameter.ComCount = 1;
  front_indicate_data_.Parameter.KickReq = false;
  front_indicate_data_.Parameter.RobotMode = 0;
  front_indicate_data_.Parameter.ChargeReq = false;
  front_indicate_data_.Parameter.ComAIPC = false;
  front_indicate_data_.Parameter.DribbleReq = false;

  for(int i = 0; i < 1 ; i++)
  {
    data[i] = *(front_indicate_data_.Indicate_data + i);   
  }

  if(i2c_write_device(pi_, i2c_handler_, data, 1) > 0)
  {
    return false;
  }

  return true;
}

