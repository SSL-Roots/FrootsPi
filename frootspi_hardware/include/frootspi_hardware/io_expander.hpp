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

#ifndef FROOTSPI_HARDWARE__IO_EXPANDER_HPP_
#define FROOTSPI_HARDWARE__IO_EXPANDER_HPP_

class IOExpander
{
public:
  IOExpander();
  ~IOExpander();

  bool open(const int pi);
  bool close();
  bool read(
    bool & pushed_button0, bool & pushed_button1, bool & pushed_button2,
    bool & pushed_button3, bool & turned_on_dip0, bool & turned_on_dip1);
  bool write(const bool set_led);

private:
  bool control_register(const char addr, const char rw, const char write_data, char * read_data);
  int spi_handler_;
  int pi_;
};

#endif  // FROOTSPI_HARDWARE__IO_EXPANDER_HPP_
