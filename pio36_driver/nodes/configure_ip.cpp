#include "modbus.hpp"
#include "blocks/ip_config.hpp"
#include "blocks/word_buffer.hpp"

class Configurator{
public:
  void connect(){
    // Prompt the user for an IP address
    std::cout << "Please enter an IP address (default 192.168.52.209): ";

    // Read the input IP address from the user
    std::getline(std::cin, ipAddress);

    if(ipAddress==""){
      ipAddress = "192.168.52.209";
    }
    // Output the entered IP address to confirm
    std::cout << "Connecting to: " << ipAddress << std::endl;

    mb_.connect_tcp(ipAddress.c_str(),502);



    printConfig();
  }

  void printConfig(){

    mb_.read_input_registers(device_config);

    std::cout << "current config:" << std::endl;

    std::cout << "IP:      " <<  device_config.data().ip_addr.toString() << std::endl;
    std::cout << "subnet:  " <<  device_config.data().subnet_mask.toString() << std::endl;
    std::cout << "gateway: " <<  device_config.data().default_gateway.toString() << std::endl;

    std::cout << std::endl;
  }

  void setIP(){

    std::string input;
    std::cout << "ip: ";
    std::getline(std::cin, input);
    user_config.data().ip_addr.setFromString(input);

    std::cout << "subnet: ";
    std::getline(std::cin, input);
    user_config.data().subnet_mask.setFromString(input);

    std::cout << "gateway: ";
    std::getline(std::cin, input);
    user_config.data().default_gateway.setFromString(input);

    mb_.write_registers(user_config);

    printConfig();
    return;
  }

  void save(){
    eprom_flush.buffer()[0]=1;
    mb_.write_registers(eprom_flush);
    std::cout << "EPROM flush set.  Settings will persist after restart" <<std::endl;
    std::cout << "run cmd 'restart' to apply IP settings" <<std::endl;
  }

  void restart(){
    restart_cmd.buffer()[0]=1;
    mb_.write_registers(restart_cmd);
    std::cout << "Restarting... program will now exit " <<std::endl;
  }

private:
  std::string ipAddress;
  pio36_driver::blocks::IpConfig user_config;
  pio36_driver::blocks::IpConfig device_config;
  pio36_driver::blocks::WordBuffer<113,1> eprom_flush;
  pio36_driver::blocks::WordBuffer<112,1> restart_cmd;
  pio36_driver::Modbus mb_;
};

int main(void) {
  Configurator conf;

  conf.connect();

  std::string cmd;



  while(cmd!="q"){
    std::cout << "enter command (q to quit):  ";
    std::getline(std::cin, cmd);

    if(cmd=="print") conf.printConfig();

    if(cmd=="setIP") conf.setIP();

    if(cmd=="save") conf.save();

    if(cmd=="restart"){
      conf.restart();
      cmd = "q";
    }

  }


  return 0;

}
