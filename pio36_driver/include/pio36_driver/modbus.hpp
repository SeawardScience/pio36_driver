#pragma once


#include "package_defs.hpp"

#include <modbus.h>
#include <cerrno>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <memory>


NS_HEAD

class Modbus{
public:
  Modbus();
  ~Modbus();
  void connect_tcp(const char *ip_address, int port);
  void read_input_registers(int addr, int nb);
  void write_registers(int addr, int nb, const uint16_t *src);
  void print_buffer();
  uint16_t * buffer();

  static const int REGISTER_MAX_WORDS = 128;
  typedef std::shared_ptr<Modbus> Ptr;
private:
  uint16_t tab_reg_[REGISTER_MAX_WORDS];
  modbus_t *mb_;
};

NS_FOOT
