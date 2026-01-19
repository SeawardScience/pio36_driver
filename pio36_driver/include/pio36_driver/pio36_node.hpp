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

/**
 * @brief Helper template for buffered ROS2 publishers
 * @tparam msg_T ROS message type
 */
template<typename msg_T>
struct BufferPub{
  void setup(rclcpp::Node * node_ptr, std::string topic) {
    pub = node_ptr->create_publisher<msg_T>(topic, 10);
  }
  typename rclcpp::Publisher<msg_T>::SharedPtr pub;
  msg_T msg_buff;  ///< Message buffer updated before publish
  void publish() {
    pub->publish(msg_buff);
  }
};

/**
 * @brief ROS2 driver node for Innova PIO36 PWM/Analog I/O board
 * 
 * Communicates with PIO36 board over Modbus TCP for hydraulic valve control.
 * The board provides 36 PWM outputs (0-100% duty cycle) and 8 analog voltage inputs.
 * Default Modbus address is 40001, default port 502.
 */
class Pio36Node : public rclcpp::Node
{
public:
  /**
   * @brief Configuration parameters for PIO36 node
   */
  struct Params {
    struct {
      int poll_ms = 500;        ///< Input register polling interval (ms)
      int connect_ms = 1000;    ///< Connection retry interval (ms)
      int write_ms = 10;        ///< Output register write interval (ms)
    } timers;
    struct {
      std::string type = "tcp";              ///< Connection type (tcp only)
      std::string ip   = "192.168.52.209";   ///< Device IP address
      int network_port = 502;                ///< Modbus TCP port
    } connection;
    struct {
      std::string valve_cmd = "/odysseus4k/control/valve";  ///< Single valve command topic
      std::string valve_pack_cmd = "cmd/valve_pack";        ///< Multi-valve command topic
      std::string valve_pack_response = "resp/valve_pack";  ///< Valve state response topic
    } topics;
    float deadband = 0.01;  ///< Minimum valve setpoint before zeroing (0-1 range)

    void declare(rclcpp::Node * node);  ///< Declare ROS parameters
    void update(rclcpp::Node * node);   ///< Update from ROS parameters
  };
  
  /** @brief Constructor - initializes timers, publishers, subscribers */
  Pio36Node();
  
  /** @brief Callback for single valve commands */
  void valveCallback(const hydraulic_interfaces::msg::ValveStamped::SharedPtr msg);
  
  /** @brief Callback for multi-valve commands */
  void valvePackCallback(const hydraulic_interfaces::msg::ValvePack::SharedPtr msg);
  
  /** @brief Timer callback to write PWM duty registers */
  void writeBuffer();

protected:
  /** @brief Process valve command, apply deadband, scale to 0-100% */
  void valveHandler(hydraulic_interfaces::msg::Valve & valve);
  
  void extracted(size_t &i);  ///< Unused helper function
  
  /** @brief Timer callback to poll input registers */
  void poll();
  
  /** @brief Publish valve states and analog voltages */
  void publishResponses();
  
  /** @brief Configure lost comms timeout and keep-value on connection */
  void sendStartupConfig();
  
  /** @brief Timer callback to establish/maintain Modbus connection */
  void connect();
  
  rclcpp::TimerBase::SharedPtr connect_timer_;  ///< Connection retry timer
  rclcpp::TimerBase::SharedPtr poll_timer_;     ///< Input polling timer
  rclcpp::TimerBase::SharedPtr write_timer_;    ///< Output write timer
  
  bool connected_ = false;      ///< Modbus connection status
  bool connected_warn = true;   ///< Flag to throttle connection warnings
  
  /** @brief Modbus input register blocks (read from device) */
  struct {
    blocks::PwmDuty pwm_duty;                       ///< Reg 1-36: PWM duty cycle feedback
    blocks::AnalogVoltageInput analog_inputs;       ///< Reg 37-44: Analog voltage inputs (0-10V)
    blocks::WordBuffer<65,1> lost_comms_timeout;    ///< Reg 65: Lost comms timeout (sec)
    blocks::WordBuffer<66,1> lost_comms_keep_value; ///< Reg 66: Keep last value on timeout (0/1)
    blocks::WordBuffer<99,1> sensor_power_config;   ///< Reg 99: Sensor power configuration
    blocks::WordBuffer<102,1> system_warnings;      ///< Reg 102: System warnings bitmask
    blocks::WordBuffer<103,1> sensor_alarms;        ///< Reg 103: Sensor alarms bitmask
    blocks::WordBuffer<104,1> pwm_faults;           ///< Reg 104: PWM fault status bitmask
  } poll_blocks_;
  
  /** @brief Modbus holding register blocks (write to device) */
  struct {
    blocks::PwmDuty pwm_duty;                       ///< Reg 1-36: PWM duty cycle setpoints (0-100)
    blocks::IpConfig ip_config;                     ///< Reg 45-52: IP configuration
    blocks::WordBuffer<113,1> eprom_flush;          ///< Reg 113: Save config to EEPROM
    blocks::WordBuffer<112,1> restart;              ///< Reg 112: Restart device
    blocks::WordBuffer<65,1> lost_comms_timeout;    ///< Reg 65: Lost comms timeout (sec)
    blocks::WordBuffer<66,1> lost_comms_keep_value; ///< Reg 66: Keep last value on timeout (0/1)
    blocks::WordBuffer<104,1> pwm_faults;           ///< Reg 104: Write 1s to clear corresponding fault bits
    blocks::WordBuffer<1033,1> pwm_ignore_faults;   ///< Reg 1033: Bitmask to enable outputs despite faults
  } cmd_blocks_;
  
  /** @brief ROS2 subscriptions */
  struct {
    rclcpp::Subscription<hydraulic_interfaces::msg::ValveStamped>::SharedPtr valve_;
    rclcpp::Subscription<hydraulic_interfaces::msg::ValvePack>::SharedPtr valve_pack_;
  } subs_;
  
  /** @brief ROS2 publishers */
  struct {
    BufferPub<hydraulic_interfaces::msg::ValvePack> valve_state;  ///< PWM duty feedback
    BufferPub<io_interfaces::msg::RawAnalogArray> voltage;        ///< Analog input voltages
  } pubs_;
  
  std::shared_ptr<Modbus> mb_;  ///< Modbus TCP connection handler
  Params params_;               ///< Node configuration parameters
};

NS_FOOT
