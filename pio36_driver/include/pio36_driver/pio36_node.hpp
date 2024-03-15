#pragma once  // Favor using this over the #ifndef, #define method

#include "blocks/pwm_duty.hpp"
#include "package_defs.hpp"
#include "modbus.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <hydraulic_interfaces/msg/valve_stamped.hpp>
#include <hydraulic_interfaces/msg/valve_pack.hpp>


NS_HEAD  // macro for consistantly defining our namespace for the package

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

    void declare(rclcpp::Node * node);
    void update(rclcpp::Node * node);
  };
  Pio36Node();
  void valveCallback(const hydraulic_interfaces::msg::ValveStamped::SharedPtr msg);
  void valvePackCallback(const hydraulic_interfaces::msg::ValvePack::SharedPtr msg);
  void writeBuffer();
protected:
  void valveHandler(hydraulic_interfaces::msg::Valve & valve);
  void poll();
  rclcpp::TimerBase::SharedPtr connect_timer_;
  void connect();
  bool connected_ = false;
  rclcpp::TimerBase::SharedPtr poll_timer_; ///< Shared pointer to the timer
  rclcpp::TimerBase::SharedPtr write_timer_;
  struct{
    blocks::PwmDuty pwm_duty;
  }poll_blocks_;
  struct{
    blocks::PwmDuty pwm_duty;
  }cmd_blocks_;
  struct{
    rclcpp::Subscription<hydraulic_interfaces::msg::ValveStamped>::SharedPtr valve_;
    rclcpp::Subscription<hydraulic_interfaces::msg::ValvePack>::SharedPtr valve_pack_;
  }subs_;
  Modbus mb_;
  Params params_;
};

NS_FOOT
