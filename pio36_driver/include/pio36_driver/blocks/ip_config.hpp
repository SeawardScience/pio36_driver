#pragma once

#include "block.hpp"


BLOCKS_NS_HEAD

/**
 * @brief Network configuration register block (Reg 45-56)
 * 
 * Stores IP address, subnet mask, and default gateway.
 * Each IPv4 address occupies 2 words (4 bytes).
 */
class IpConfig : public Block{

public:
  static const size_t NUM_PWM_CHANNELS = 16;
  
  /** @brief IPv4 address storage and conversion */
  struct IPv4 {
    //std::array<uint8_t, 4> octets; // Array to store the octets
    uint8_t octet2;
    uint8_t octet3;
    uint8_t octet0;
    uint8_t octet1;

    /** @brief Parse IPv4 address from dot-decimal string (e.g., "192.168.1.100") */
    void setFromString(const std::string& ipString) {
      std::istringstream ipStream(ipString);
      int o1, o2, o3, o4; // Temporary variables to store the octets as integers
      char dot; // To temporarily store the dots

      if (ipStream >> o1 >> dot >> o2 >> dot >> o3 >> dot >> o4) {
        octet0 = o1;
        octet1 = o2;
        octet2 = o3;
        octet3 = o4;
      } else {
        //std::cerr << "Invalid IP format" << std::endl;
      }
    }
    /** @brief Convert IPv4 address to dot-decimal string */
    std::string toString() const {
      std::ostringstream ipStream;
      ipStream << static_cast<int>(octet0) << "."
               << static_cast<int>(octet1) << "."
               << static_cast<int>(octet2) << "."
               << static_cast<int>(octet3);
      return ipStream.str();
    }
  };

  /** @brief Complete network configuration structure */
  struct IpConfig_t{
    IPv4 ip_addr;
    IPv4 subnet_mask;
    IPv4 default_gateway;
  };

  static const size_t BLOCK_SIZE = sizeof(IpConfig_t)/2;

protected:
  union{
    IpConfig_t type;
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  //std::shared_ptr<PwmDuty> SharedPtr;
  IpConfig(){
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i]=0;
    }};
  primitives::WordBuffer buffer(){return data_.word_buffer;}
  size_t size(){return BLOCK_SIZE;}

  /** @brief Access network configuration structure */
  IpConfig_t & data(){return data_.type;}

  /*!
   * \brief innovaAddress returns the of the PWM block in the innova documentation
   * \return
   */
  int innovaAddress(){return 50;}
};

BLOCKS_NS_FOOT
