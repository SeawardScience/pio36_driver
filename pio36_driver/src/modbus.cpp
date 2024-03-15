#include "modbus.hpp"

NS_HEAD


Modbus::Modbus()
{
  for(size_t i = 0 ; i<REGISTER_MAX_WORDS ; i++){
    tab_reg_[i]=0;
  }
}

Modbus::~Modbus(){
  modbus_close(mb_);
  modbus_free(mb_);
}

void Modbus::connect_tcp(const char *ip_address, int port){
  mb_ =modbus_new_tcp(ip_address, port);
  if (modbus_connect(mb_) == -1) {
    throw std::runtime_error("Connection failed: " + std::string(modbus_strerror(errno)));
  }
}

void Modbus::read_input_registers(int addr, int nb){
  if ( modbus_read_input_registers(mb_, addr, nb, tab_reg_) == -1){
    throw std::runtime_error("Failed to read input registers: " + std::string(modbus_strerror(errno)));
  }
}

void Modbus::read_input_registers(blocks::Block & block)
{
  if ( modbus_read_input_registers(mb_, block.modbusAddress(), block.size(), block.buffer()) == -1){
    throw std::runtime_error("Failed to read input registers: " + std::string(modbus_strerror(errno)));
  }
}

void Modbus::write_registers(int addr, int nb, const uint16_t *src){
  if ( modbus_write_registers(mb_, addr, nb, src) == -1){
    throw std::runtime_error("Failed to write registers: " + std::string(modbus_strerror(errno)));
  }
}

void Modbus::write_registers(blocks::Block & block)
{
  if ( modbus_write_registers(mb_, block.modbusAddress(), block.size(), block.buffer()) == -1){
    throw std::runtime_error("Failed to write registers: " + std::string(modbus_strerror(errno)));
  }
}

void Modbus::print_buffer(){
  for (int i = 0; i < REGISTER_MAX_WORDS; ++i) {
      std::cout << "0x"
                << std::hex << std::uppercase // Hexadecimal format, uppercase letters
                << std::setw(4) << std::setfill('0') // Set width to 4, fill with '0'
                << tab_reg_[i];

      if (i < REGISTER_MAX_WORDS - 1) {
          std::cout << ", "; // Separate the values with a comma
      }
  }

  std::cout << std::endl;
}

uint16_t *Modbus::buffer()
{
 return tab_reg_;
}


NS_FOOT




