// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERBOTIX_SLATE_DRIVER__SERIAL_DRIVER_HPP_
#define INTERBOTIX_SLATE_DRIVER__SERIAL_DRIVER_HPP_

#include <string>
#define ASSERT(expr, fail) (static_cast<bool>(expr) ? void(0) : (fail))

class SerialDriver
{
public:
  bool init(const std::string & portname, std::string & dev, int baudRate);
  void close();

  bool setEntry(int index, const float * data_address);
  bool setEntry(int index, const int * data_address);
  bool setEntry(int index, const char * data_address);

  bool getEntry(int index, float * data_address);
  bool getEntry(int index, int * data_address);
  bool getEntry(int index, char * data_address);

private:
  bool handleTimeout(int);

  int fd_;
  bool isOpened_;
  int conn_timeout_cnt_;
  int baud_rate_ = 0;
  std::string portname_;
};

#endif  // INTERBOTIX_SLATE_DRIVER__SERIAL_DRIVER_HPP_
