#ifndef __SERIAL_DRIVER_H_
#define __SERIAL_DRIVER_H_

#include <string>
#define ASSERT(expr, fail) (static_cast<bool>(expr) ? void(0) : (fail))

class SerialDriver
{
 public:
  bool init(const std::string& portname, std::string& dev, int baudRate);
  void close();

  bool setEntry(int index, const float* data_address);
  bool setEntry(int index, const int* data_address);
  bool setEntry(int index, const char* data_address);

  bool getEntry(int index, float* data_address);
  bool getEntry(int index, int* data_address);
  bool getEntry(int index, char* data_address);

 private:
  bool handleTimeout(int);

  int fd_;
  bool isOpened_;
  int conn_timeout_cnt_;
  int baud_rate_ = 0;
  std::string portname_;
};

#endif  // __SERIAL_DRIVER_H_
