/** Copyright © 2025 Seaward Science. */

#include "pio36_node.hpp"
#include <cstring>

using namespace std::chrono_literals;
using std::placeholders::_1;
NS_HEAD

Pio36Node::Pio36Node()
    : Node("pio36")
{
  params_.declare(this);
  params_.update(this);
  poll_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.poll_ms), std::bind(&Pio36Node::poll, this));
  connect_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.connect_ms), std::bind(&Pio36Node::connect, this));

  subs_.valve_ = this->create_subscription<hydraulic_interfaces::msg::ValveStamped>(
      params_.topics.valve_cmd, 24, std::bind(&Pio36Node::valveCallback, this, _1));

  subs_.valve_pack_ = this->create_subscription<hydraulic_interfaces::msg::ValvePack>(
      params_.topics.valve_pack_cmd, 1, std::bind(&Pio36Node::valvePackCallback, this, _1));

  subs_.dither_ = this->create_subscription<pio36_interfaces::msg::DitherPack>(
      params_.topics.dither_cmd, 1, std::bind(&Pio36Node::ditherCallback, this, _1));

  write_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.write_ms), std::bind(&Pio36Node::writeBuffer, this));

  pubs_.valve_state.setup(this, "valve_state");
  pubs_.valve_state.msg_buff.valves.resize(blocks::PwmDuty::NUM_PWM_CHANNELS);

  pubs_.voltage.setup(this, "analog_input_voltage");
  pubs_.voltage.msg_buff.analogs.resize(blocks::AnalogVoltageInput::NUM_CHANNELS);

  pubs_.dither_state.setup(this, "dither_state");
  pubs_.dither_state.msg_buff.dithers.resize(blocks::PwmDuty::NUM_PWM_CHANNELS);

  // initialize commmands with non zero values so they take effect
  cmd_blocks_.eprom_flush.buffer()[0] = 1;
  cmd_blocks_.restart.buffer()[0] = 1;
  cmd_blocks_.lost_comms_timeout.buffer()[0] = 5;
  cmd_blocks_.lost_comms_keep_value.buffer()[0] = 0;
  cmd_blocks_.pwm_ignore_faults.buffer()[0] = 0x0000;  // Enable fault detection on all channels
  //cmd_blocks_.sensor_power_config.buffer()[0] = 0x01;  // Bit 0: Ignore sensor power fault
}

void Pio36Node::valveHandler(hydraulic_interfaces::msg::Valve & valve) {
  if (valve.valve_id < 1 || valve.valve_id > blocks::PwmDuty::NUM_PWM_CHANNELS) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Valve id %u out of range (1..%zu). Ignoring command.",
      valve.valve_id, blocks::PwmDuty::NUM_PWM_CHANNELS);
    return;
  }
  auto value = valve.set_point * 100.0;
  if (value > 100.0) {
    value = 100.0;
  }
  if (value < params_.deadband) {
    value = 0.0;
  }
  if (value < 0.0) {
    RCLCPP_WARN_ONCE(this->get_logger(), "This board does not support negative valve commands, "
                     "received a negative value for thrust on channel %i", valve.valve_id);
  }
  RCLCPP_DEBUG(this->get_logger(), "Setting valve %i to %.2f%%", valve.valve_id, value);
  cmd_blocks_.pwm_duty.dutyChan(valve.valve_id) = value;
}

void Pio36Node::valveCallback(const hydraulic_interfaces::msg::ValveStamped::SharedPtr msg) {
  valveHandler(msg->valve);
}

void Pio36Node::valvePackCallback(const hydraulic_interfaces::msg::ValvePack::SharedPtr msg) {
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received valve pack message with %zu valves", msg->valves.size());
  for (auto valve : msg->valves) {
    valveHandler(valve);
  }
}

void Pio36Node::ditherCallback(const pio36_interfaces::msg::DitherPack::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received dither pack with %zu channels", msg->dithers.size());
  for (const auto& dither : msg->dithers) {
    if (dither.channel_id < 1 || dither.channel_id > blocks::PwmDuty::NUM_PWM_CHANNELS) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dither channel_id %d out of range (1..%zu). Ignoring.",
        dither.channel_id, blocks::PwmDuty::NUM_PWM_CHANNELS);
      continue;
    }
    
    // Set dither parameters for this channel
    cmd_blocks_.dither_duty.ditherDutyChan(dither.channel_id) = dither.dither_duty;
    cmd_blocks_.dither_frequency.ditherFreqChan(dither.channel_id) = dither.dither_frequency;
    cmd_blocks_.dither_phase.ditherPhaseChan(dither.channel_id) = dither.dither_phase;
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Set channel %d dither: duty=%.2f%%, freq=%.2fHz, phase=%.1f°",
                 dither.channel_id, dither.dither_duty, dither.dither_frequency, dither.dither_phase);
  }
  
  // Write dither registers immediately
  if (connected_) {
    try {
      mb_->write_registers(cmd_blocks_.dither_duty);
      mb_->write_registers(cmd_blocks_.dither_frequency);
      mb_->write_registers(cmd_blocks_.dither_phase);
      RCLCPP_DEBUG(this->get_logger(), "Dither parameters written to device");
    } catch (const std::runtime_error& err) {
      RCLCPP_WARN(this->get_logger(), "Failed to write dither parameters: %s", err.what());
    }
  }
}

void Pio36Node::writeBuffer() {
  if (connected_) {
    try {
      mb_->write_registers(cmd_blocks_.pwm_duty);
    } catch (const std::runtime_error& err) {
      std::string error_msg = err.what();
      RCLCPP_WARN(this->get_logger(), "modbus write failed: %s", error_msg.c_str());
      connected_ = false;
    }
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                         "Cannot write - not connected to device");
  }
}

void Pio36Node::poll() {
  if (connected_) {
    try {
      mb_->read_input_registers(poll_blocks_.pwm_duty);
      mb_->read_input_registers(poll_blocks_.analog_inputs);
      mb_->read_input_registers(poll_blocks_.dither_duty);
      mb_->read_input_registers(poll_blocks_.dither_frequency);
      mb_->read_input_registers(poll_blocks_.dither_phase);
      publishResponses();
    } catch (const std::runtime_error& err) {
      std::string error_msg = err.what();
      RCLCPP_WARN(this->get_logger(), "modbus read failed: %s", error_msg.c_str());
      connected_ = false;
    }
  }
}

void Pio36Node::publishResponses() {
  for (size_t i = 1; i <= blocks::PwmDuty::NUM_PWM_CHANNELS; i++) {
    pubs_.valve_state.msg_buff.valves[i-1].set_point = static_cast<double>(poll_blocks_.pwm_duty.dutyChan(i)) / 100.0;
    pubs_.valve_state.msg_buff.valves[i-1].valve_id = i;
  }
  pubs_.valve_state.publish();
  
  for (size_t i = 1; i <= blocks::AnalogVoltageInput::NUM_CHANNELS; i++) {
    pubs_.voltage.msg_buff.analogs[i-1].proportional_value =
        static_cast<double>(poll_blocks_.analog_inputs.channel(i))/10.0;
    pubs_.voltage.msg_buff.analogs[i-1].scale = 10.0;
    pubs_.voltage.msg_buff.analogs[i-1].channel_id = i;
  }
  pubs_.voltage.publish();
  
  pubs_.dither_state.msg_buff.header.stamp = this->now();
  pubs_.dither_state.msg_buff.header.frame_id = "";
  for (size_t i = 1; i <= blocks::PwmDuty::NUM_PWM_CHANNELS; i++) {
    auto& dither = pubs_.dither_state.msg_buff.dithers[i-1];
    dither.channel_id = i;
    dither.dither_duty = static_cast<float>(poll_blocks_.dither_duty.ditherDutyChan(i));
    dither.dither_frequency = static_cast<float>(poll_blocks_.dither_frequency.ditherFreqChan(i));
    dither.dither_phase = static_cast<float>(poll_blocks_.dither_phase.ditherPhaseChan(i));
  }
  pubs_.dither_state.publish();
}

void Pio36Node::sendStartupConfig()
{
  try {
    mb_->write_registers(cmd_blocks_.lost_comms_timeout);
    mb_->write_registers(cmd_blocks_.lost_comms_keep_value);

    mb_->read_input_registers(poll_blocks_.lost_comms_timeout);
    mb_->read_input_registers(poll_blocks_.lost_comms_keep_value);
    RCLCPP_INFO(this->get_logger(), "Config set to, timeout: %i , keep value %i",
                poll_blocks_.lost_comms_timeout.buffer()[0],
                poll_blocks_.lost_comms_keep_value.buffer()[0]);

    // Read board diagnostics
    mb_->read_input_registers(poll_blocks_.system_warnings);
    mb_->read_input_registers(poll_blocks_.sensor_alarms);
    mb_->read_input_registers(poll_blocks_.pwm_faults);
    uint16_t warnings = poll_blocks_.system_warnings.buffer()[0];
    uint16_t alarms = poll_blocks_.sensor_alarms.buffer()[0];
    uint16_t faults = poll_blocks_.pwm_faults.buffer()[0];

    RCLCPP_INFO(this->get_logger(), "Board status - Warnings: 0x%04X, Alarms: 0x%04X, PWM Faults: 0x%04X", 
                warnings, alarms, faults);
    
    // Read initial PWM frequencies
    mb_->read_input_registers(poll_blocks_.pwm_freq_1_4);
    mb_->read_input_registers(poll_blocks_.pwm_freq_5_8);
    mb_->read_input_registers(poll_blocks_.pwm_freq_9_12);
    mb_->read_input_registers(poll_blocks_.pwm_freq_13_16);
    
    // Convert from big-endian word buffer to float
    primitives::BE_f32 freq_1_4, freq_5_8, freq_9_12, freq_13_16;
    std::memcpy(&freq_1_4, poll_blocks_.pwm_freq_1_4.buffer(), sizeof(float));
    std::memcpy(&freq_5_8, poll_blocks_.pwm_freq_5_8.buffer(), sizeof(float));
    std::memcpy(&freq_9_12, poll_blocks_.pwm_freq_9_12.buffer(), sizeof(float));
    std::memcpy(&freq_13_16, poll_blocks_.pwm_freq_13_16.buffer(), sizeof(float));
    
    RCLCPP_INFO(this->get_logger(), "PWM Frequencies - Ch1-4: %.1fHz, Ch5-8: %.1fHz, Ch9-12: %.1fHz, Ch13-16: %.1fHz",
                static_cast<float>(freq_1_4), static_cast<float>(freq_5_8), 
                static_cast<float>(freq_9_12), static_cast<float>(freq_13_16));
    
    if (warnings != 0) {
      decodeBitmask(warnings, SYSTEM_WARNING_BITS, 
                    sizeof(SYSTEM_WARNING_BITS)/sizeof(SYSTEM_WARNING_BITS[0]), "System Warning");
    }
    if (alarms != 0) {
      decodeBitmask(alarms, SENSOR_ALARM_BITS, 
                    sizeof(SENSOR_ALARM_BITS)/sizeof(SENSOR_ALARM_BITS[0]), "Sensor Alarm");
    }
    
    // Clear all sensor alarms to enable PWM system
    cmd_blocks_.sensor_alarms.buffer()[0] = 0xFFFF;
    mb_->write_registers(cmd_blocks_.sensor_alarms);
    RCLCPP_INFO(this->get_logger(), "Cleared sensor alarms");

    RCLCPP_INFO(this->get_logger(), "PIO36 initialization complete");
  } catch (const std::runtime_error& err) {
    std::string error_msg = err.what();
    RCLCPP_WARN(this->get_logger(), "Failed to write startup config: %s", error_msg.c_str());
    connected_ = false;
  }
}

void Pio36Node::connect()
{
  if (connected_) {
    return;
    connected_warn = true;
  }
  try {
    mb_.reset(new Modbus);
    if (params_.connection.type == "tcp") {
      RCLCPP_INFO_ONCE(this->get_logger(), "Connecting to %s:%i",
                       params_.connection.ip.c_str(), params_.connection.network_port);
      mb_->connect_tcp(params_.connection.ip.c_str(), params_.connection.network_port);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid connection type specified %s",
                   params_.connection.type.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Connected to device");
    connected_ = true;
    sendStartupConfig();
  } catch (const std::runtime_error& err) {
    connected_ = false;
    std::string error_msg = err.what();
    if (connected_warn) {
      RCLCPP_WARN(this->get_logger(), "Unable to connect to %s: %s",
                  params_.connection.ip.c_str(), error_msg.c_str());
    }
    connected_warn = false;
  }
}

void Pio36Node::Params::declare(rclcpp::Node *node)
{
  // Declare parameters for the 'timers' struct
  node->declare_parameter("timers.poll_ms", timers.poll_ms);
  node->declare_parameter("timers.connect_ms", timers.connect_ms);
  node->declare_parameter("timers.write_ms", timers.write_ms);

  // Declare parameters for the 'connection' struct
  node->declare_parameter("connection.type", connection.type);
  node->declare_parameter("connection.ip", connection.ip);
  node->declare_parameter("connection.network_port", connection.network_port);

  // Declare parameters for the 'topics' struct
  node->declare_parameter("topics.valve_cmd", topics.valve_cmd);
  node->declare_parameter("topics.valve_pack_cmd", topics.valve_pack_cmd);
  node->declare_parameter("topics.valve_pack_response", topics.valve_pack_response);
  node->declare_parameter("topics.dither_cmd", topics.dither_cmd);

  node->declare_parameter("deadband", deadband);
}

void Pio36Node::Params::update(rclcpp::Node *node)
{
  // Update parameters for the 'timers' struct
  node->get_parameter("timers.poll_ms", timers.poll_ms);
  node->get_parameter("timers.connect_ms", timers.connect_ms);
  node->get_parameter("timers.write_ms", timers.write_ms);

  // Update parameters for the 'connection' struct
  node->get_parameter("connection.type", connection.type);
  node->get_parameter("connection.ip", connection.ip);
  node->get_parameter("connection.network_port", connection.network_port);

  // Update parameters for the 'topics' struct
  node->get_parameter("topics.valve_cmd", topics.valve_cmd);
  node->get_parameter("topics.valve_pack_cmd", topics.valve_pack_cmd);
  node->get_parameter("topics.valve_pack_response", topics.valve_pack_response);
  node->get_parameter("topics.dither_cmd", topics.dither_cmd);

  node->get_parameter("deadband", deadband);
}

void Pio36Node::decodeBitmask(uint16_t bitmask, const BitDescription* bits, size_t count, const char* type) {
  for (size_t i = 0; i < count; i++) {
    if (bitmask & (1 << bits[i].bit)) {
      RCLCPP_WARN(this->get_logger(), "%s: %s", type, bits[i].description);
    }
  }
}

NS_FOOT
