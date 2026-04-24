#include <stdio.h>
#include <modbus.h>
#include <cerrno>
#include <chrono>
#include <thread>
#include "modbus.hpp"
#include "blocks/primatives.hpp"


union FloatToUint16 {
    float value;
    uint16_t parts[2];
};

int main(void) {

  pio36_driver::Modbus mb;
  mb.connect_tcp("192.168.52.209", 502);
  mb.read_input_registers(2,2);
  mb.print_buffer();

  uint32_t value = (mb.buffer()[0] << 16) + mb.buffer()[1];
  printf("Combined 32-bit value: %u\n", value);

  uint16_t buffer[2];

  FloatToUint16  u;
  u.value = 1.0;

  buffer[0] = u.parts[1];
  buffer[1] = u.parts[0];
  union{
    pio36_driver::primitives::BE_f32 val;
    uint16_t parts[2];
  } pwm;
  pwm.val.set(1.0);

  mb.write_registers(117-1,2,buffer);

  while(true){
    u.value = u.value + 1.0;
    if(u.value > 100.0){
      u.value = 0.0;
    }

    buffer[0] = u.parts[1];
    buffer[1] = u.parts[0];


    mb.write_registers(1000-1,2,buffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    mb.read_input_registers(1000-1,2);
    mb.print_buffer();
    std::cout << modbus_get_float_abcd(mb.buffer()) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  return 0;

}
