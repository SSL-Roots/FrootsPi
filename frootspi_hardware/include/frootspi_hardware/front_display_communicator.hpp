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

#ifndef FROOTSPI_HARDWARE__FRONT_DISPLAY_COMMUNICATOR_HPP_
#define FROOTSPI_HARDWARE__FRONT_DISPLAY_COMMUNICATOR_HPP_

#include <string>

union FrontIndicateData{
    char Indicate_data[5];
    struct{
        char    RobotID         :4;
        char    RobotMode       :2;
        bool    ComAIPC         :1;
        bool    BallSens        :1;
        unsigned char    BatVol          :8;
        unsigned char    CapVol          :8;
        unsigned char    ComCount        :8;
        bool    KickReq         :1;
        bool    DribbleReq      :1;
        bool    ChargeReq       :1;
        bool    CapacitorSta    :1;
        char    Dummy           :4;
    }Parameter;
};

class FrontDisplayCommunicator
{
public:
  FrontDisplayCommunicator();
  ~FrontDisplayCommunicator();

  bool open(const int pi);
  bool close();
  bool send_data(void);

private:
 
  int i2c_handler_;
  int pi_;
  FrontIndicateData front_indicate_data_;
};

#endif  // FROOTSPI_HARDWARE__FRONT_DISPLAY_COMMUNICATOR_HPP_