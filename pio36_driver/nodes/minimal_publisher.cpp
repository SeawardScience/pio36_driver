#include <stdio.h>
#include <modbus.h>
#include <cerrno>

int main(void) {
  modbus_t *mb;
  uint16_t tab_reg[128];

  for(size_t i = 0 ; i<32 ; i++){
    tab_reg[i]=0;
  }

  mb = modbus_new_tcp("192.168.52.209", 502);
  if (modbus_connect(mb) == -1) {
    fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(mb);
    return -1;
  }

  if ( modbus_read_input_registers(mb, 2, 2, tab_reg) == -1){
    fprintf(stderr, "failed to read: %s", modbus_strerror(errno));
  }

  uint32_t value = (tab_reg[0] << 16) + tab_reg[1];

  printf("Combined 32-bit value: %u\n", value);
  modbus_close(mb);
  modbus_free(mb);

}
