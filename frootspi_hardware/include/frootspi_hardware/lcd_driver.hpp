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

#ifndef FROOTSPI_HARDWARE__LCD_DRIVER_HPP_
#define FROOTSPI_HARDWARE__LCD_DRIVER_HPP_

#include <string>

class LCDDriver
{
public:
  LCDDriver();
  ~LCDDriver();

  bool open(const int pi);
  bool close();
  bool write_texts(const std::string text1, const std::string text2);

private:
  bool init_device();
  bool write_command(const unsigned command);
  bool write_data(const unsigned data);
  bool set_function(
    const bool bus_8bit, const bool disp_2line,
    const bool double_height_font, const unsigned instruction_table);
  bool set_osc_freq(const bool bias, const bool f2, const bool f1, const bool f0);
  bool set_contrast_lowbits(const bool c3, const bool c2, const bool c1, const bool c0);
  bool set_contrast_highbits(
    const bool icon_display_on, const bool booster_on,
    const bool c5, const bool c4);
  bool set_contrast(
    const bool icon_display_on, const bool booster_on,
    const unsigned contrast);
  bool set_follower_control(
    const bool follower_on, const bool rab2,
    const bool rab1, const bool rab0);
  bool turn_on_display(
    const bool display_on, const bool cursor_on,
    const bool cursor_blink_on);
  bool clear_display();
  bool set_address(const unsigned address);
  void write_line(const std::string text);

  int i2c_handler_;
  int pi_;
};

#endif  // FROOTSPI_HARDWARE__LCD_DRIVER_HPP_
