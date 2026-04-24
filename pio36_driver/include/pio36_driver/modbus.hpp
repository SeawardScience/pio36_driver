#pragma once


#include "package_defs.hpp"
#include "blocks/block.hpp"

#include <modbus.h>
#include <cerrno>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <memory>


NS_HEAD

/**
 * @brief Wrapper class for libmodbus TCP communication
 */
class Modbus{
public:
  /** @brief Constructor - initializes register buffer */
  Modbus();
  
  /** @brief Destructor - closes connection and frees resources */
  ~Modbus();
  
  /** @brief Connect to Modbus TCP device */
  void connect_tcp(const char *ip_address, int port);
  
  /** @brief Read input registers into internal buffer */
  void read_input_registers(int addr, int nb);
  
  /** @brief Read input registers into Block object */
  void read_input_registers(blocks::Block & block);
  
  /** @brief Write holding registers from buffer */
  void write_registers(int addr, int nb, const uint16_t *src);
  
  /** @brief Write holding registers from Block object */
  void write_registers(blocks::Block & block);
  
  /** @brief Print register buffer in hex format */
  void print_buffer();
  
  /** @brief Get pointer to internal register buffer */
  uint16_t * buffer();

  static const int REGISTER_MAX_WORDS = 128;  ///< Max buffer size
  typedef std::shared_ptr<Modbus> Ptr;
  
private:
  uint16_t tab_reg_[REGISTER_MAX_WORDS];  ///< Register buffer
  modbus_t *mb_;  ///< libmodbus context
};

NS_FOOT
