#pragma once  // Favor using this over the #ifndef, #define method

#include "blocks/ip_config.hpp"
#include "blocks/pwm_duty.hpp"
#include "blocks/word_buffer.hpp"
#include "blocks/analog_input_scaled.hpp"
#include "package_defs.hpp"
#include "modbus.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <hydraulic_interfaces/msg/valve_stamped.hpp>
#include <hydraulic_interfaces/msg/valve_pack.hpp>
#include <io_interfaces/msg/raw_analog_array.hpp>


NS_HEAD  // macro for consistantly defining our namespace for the package

template<typename msg_T>
struct BufferPub{
  void setup(rclcpp::Node * node_ptr, std::string topic){
    pub = node_ptr->create_publisher<msg_T>(topic, 10);
  }
  typename rclcpp::Publisher<msg_T>::SharedPtr pub;
  msg_T msg_buff;
  void publish(){
    pub->publish(msg_buff);
  }
};

class Pio36Node : public rclcpp::Node
{
public:
  struct Params{
    struct{
      int poll_ms = 500;
      int connect_ms = 1000;
      int write_ms = 10;
    }timers;
    struct{
      std::string type = "tcp";
      std::string ip   = "192.168.52.209";
      int network_port = 502;
    }connection;
    struct{
      std::string valve_cmd = "/odysseus4k/control/valve";
      std::string valve_pack_cmd = "cmd/valve_pack";
      std::string valve_pack_response = "resp/valve_pack";
    }topics;
    float deadband = 0.01;

    void declare(rclcpp::Node * node);
    void update(rclcpp::Node * node);
  };
  Pio36Node();
  void valveCallback(const hydraulic_interfaces::msg::ValveStamped::SharedPtr msg);
  void valvePackCallback(const hydraulic_interfaces::msg::ValvePack::SharedPtr msg);
  void writeBuffer();

protected:
  void valveHandler(hydraulic_interfaces::msg::Valve & valve);
  void extracted(size_t &i);
  void poll();
  void publishResponses();
  void sendStartupConfig();
  rclcpp::TimerBase::SharedPtr connect_timer_;
  void connect();
  bool connected_ = false;
  bool connected_warn = true;
  rclcpp::TimerBase::SharedPtr poll_timer_; ///< Shared pointer to the timer
  rclcpp::TimerBase::SharedPtr write_timer_;
  struct{
    blocks::PwmDuty pwm_duty;
    blocks::AnalogVoltageInput analog_inputs;
    blocks::WordBuffer<65,1> lost_comms_timeout;
    blocks::WordBuffer<66,1> lost_comms_keep_value;
  }poll_blocks_;
  struct{
    blocks::PwmDuty pwm_duty;
    blocks::IpConfig ip_config;
    blocks::WordBuffer<113,1> eprom_flush;
    blocks::WordBuffer<112,1> restart;
    blocks::WordBuffer<65,1> lost_comms_timeout;
    blocks::WordBuffer<66,1> lost_comms_keep_value;
  }cmd_blocks_;
  struct{
    rclcpp::Subscription<hydraulic_interfaces::msg::ValveStamped>::SharedPtr valve_;
    rclcpp::Subscription<hydraulic_interfaces::msg::ValvePack>::SharedPtr valve_pack_;
  }subs_;
  struct{
    BufferPub<hydraulic_interfaces::msg::ValvePack> valve_state;
    BufferPub<io_interfaces::msg::RawAnalogArray> voltage;
  }pubs_;
  std::shared_ptr<Modbus> mb_;
  Params params_;
};

NS_FOOT
