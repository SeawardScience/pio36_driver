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

  write_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.write_ms), std::bind(&Pio36Node::writeBuffer, this));
  diagnostics_timer_ = this->create_wall_timer(1000ms, std::bind(&Pio36Node::publishDiagnostics, this));

  pubs_.valve_state.setup(this, "valve_state");
  pubs_.valve_state.msg_buff.valves.resize(blocks::PwmDuty::NUM_PWM_CHANNELS);

  pubs_.voltage.setup(this, "analog_input_voltage");
  pubs_.voltage.msg_buff.analogs.resize(blocks::AnalogVoltageInput::NUM_CHANNELS);

  pubs_.diagnostics.setup(this, "diagnostics");

  // Register parameter update callback for runtime tuning
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Pio36Node::onParameterUpdate, this, std::placeholders::_1));

  // initialize commmands with non zero values so they take effect
  cmd_blocks_.eprom_flush.buffer()[0] = 1;
  cmd_blocks_.restart.buffer()[0] = 1;
  cmd_blocks_.lost_comms_timeout.buffer()[0] = params_.lost_comms_timeout;
  cmd_blocks_.lost_comms_keep_value.buffer()[0] = 0; // 0: set outputs to 0 on timeout
  cmd_blocks_.pwm_ignore_faults.buffer()[0] = 0x0000;  // Enable fault detection on all channels
  //cmd_blocks_.sensor_power_config.buffer()[0] = 0x01;  // Bit 0: Ignore sensor power fault
}

void Pio36Node::valveHandler(hydraulic_interfaces::msg::Valve & valve) {
  if (valve.valve_id < 1 || static_cast<size_t>(valve.valve_id) > blocks::PwmDuty::NUM_PWM_CHANNELS) {
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

void Pio36Node::applyDither() {
  // Apply global dither settings to all channels
  for (size_t i = 1; i <= blocks::PwmDuty::NUM_PWM_CHANNELS; i++) {
    cmd_blocks_.dither_duty.ditherDutyChan(i) = params_.dither.duty;
    cmd_blocks_.dither_frequency.ditherFreqChan(i) = params_.dither.frequency;
    cmd_blocks_.dither_phase.ditherPhaseChan(i) = params_.dither.phase;
  }
  
  // Write dither registers to device
  try {
    mb_->write_registers(cmd_blocks_.dither_duty);
    mb_->write_registers(cmd_blocks_.dither_frequency);
    mb_->write_registers(cmd_blocks_.dither_phase);

    // Read back from board to confirm values took effect
    mb_->read_input_registers(poll_blocks_.dither_duty);
    mb_->read_input_registers(poll_blocks_.dither_frequency);
    mb_->read_input_registers(poll_blocks_.dither_phase);
    RCLCPP_INFO(this->get_logger(),
                "Dither params confirmed on board (ch1): duty=%.2f%%, freq=%.2fHz, phase=%.1f°",
                static_cast<double>(poll_blocks_.dither_duty.ditherDutyChan(1)),
                static_cast<double>(poll_blocks_.dither_frequency.ditherFreqChan(1)),
                static_cast<double>(poll_blocks_.dither_phase.ditherPhaseChan(1)));
  } catch (const std::runtime_error& err) {
    RCLCPP_WARN(this->get_logger(), "Failed to write dither parameters: %s", err.what());
    throw;
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

      // Read diagnostics for continuous monitoring
      mb_->read_input_registers(poll_blocks_.system_warnings);
      mb_->read_input_registers(poll_blocks_.sensor_alarms);
      
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
}

void Pio36Node::publishDiagnostics() {
  uint16_t warnings = poll_blocks_.system_warnings.buffer()[0];
  uint16_t alarms = poll_blocks_.sensor_alarms.buffer()[0];
  
  // Log changes
  if (warnings != prev_warnings_) {
    RCLCPP_INFO(this->get_logger(), "System Warnings changed: 0x%04X -> 0x%04X", prev_warnings_, warnings);
    if (warnings != 0) {
      decodeBitmask(warnings, SYSTEM_WARNING_BITS, 
                    sizeof(SYSTEM_WARNING_BITS)/sizeof(SYSTEM_WARNING_BITS[0]), "System Warning");
    }
  }
  if (alarms != prev_alarms_) {
    RCLCPP_INFO(this->get_logger(), "Sensor Alarms changed: 0x%04X -> 0x%04X", prev_alarms_, alarms);
    if (alarms != 0) {
      decodeBitmask(alarms, SENSOR_ALARM_BITS, 
                    sizeof(SENSOR_ALARM_BITS)/sizeof(SENSOR_ALARM_BITS[0]), "Sensor Alarm");
    }
  }
  
  // Update previous state
  prev_warnings_ = warnings;
  prev_alarms_ = alarms;
  
  // Build DiagnosticArray message
  pubs_.diagnostics.msg_buff.status.clear();
  pubs_.diagnostics.msg_buff.header.stamp = this->now();
  
  // System Warnings Status
  diagnostic_msgs::msg::DiagnosticStatus warning_status;
  warning_status.name = "PIO36 System Warnings";
  warning_status.hardware_id = params_.connection.ip;
  
  if (warnings == 0) {
    warning_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    warning_status.message = "No system warnings";
  } else {
    warning_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    warning_status.message = "System warnings detected";
    
    // Add detailed key-value pairs
    for (size_t i = 0; i < sizeof(SYSTEM_WARNING_BITS)/sizeof(SYSTEM_WARNING_BITS[0]); i++) {
      if (warnings & (1 << SYSTEM_WARNING_BITS[i].bit)) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = std::string("Bit ") + std::to_string(SYSTEM_WARNING_BITS[i].bit);
        kv.value = SYSTEM_WARNING_BITS[i].description;
        warning_status.values.push_back(kv);
      }
    }
  }
  pubs_.diagnostics.msg_buff.status.push_back(warning_status);
  
  // Sensor Alarms Status
  diagnostic_msgs::msg::DiagnosticStatus alarm_status;
  alarm_status.name = "PIO36 Sensor Alarms";
  alarm_status.hardware_id = params_.connection.ip;
  
  if (alarms == 0) {
    alarm_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    alarm_status.message = "No sensor alarms";
  } else {
    alarm_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    alarm_status.message = "Sensor alarms detected";
    
    // Add detailed key-value pairs
    for (size_t i = 0; i < sizeof(SENSOR_ALARM_BITS)/sizeof(SENSOR_ALARM_BITS[0]); i++) {
      if (alarms & (1 << SENSOR_ALARM_BITS[i].bit)) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = std::string("Bit ") + std::to_string(SENSOR_ALARM_BITS[i].bit);
        kv.value = SENSOR_ALARM_BITS[i].description;
        alarm_status.values.push_back(kv);
      }
    }
  }
  pubs_.diagnostics.msg_buff.status.push_back(alarm_status);
  
  // Publish diagnostic array
  pubs_.diagnostics.publish();
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
    
    // Apply global dither configuration to all channels
    applyDither();

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

  // Declare global dither parameters
  node->declare_parameter("dither.duty", dither.duty);
  node->declare_parameter("dither.frequency", dither.frequency);
  node->declare_parameter("dither.phase", dither.phase);

  node->declare_parameter("lost_comms_timeout", lost_comms_timeout);
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

  // Update global dither parameters
  node->get_parameter("dither.duty", dither.duty);
  node->get_parameter("dither.frequency", dither.frequency);
  node->get_parameter("dither.phase", dither.phase);

  node->get_parameter("lost_comms_timeout", lost_comms_timeout);
  node->get_parameter("deadband", deadband);
}

void Pio36Node::decodeBitmask(uint16_t bitmask, const BitDescription* bits, size_t count, const char* type) {
  for (size_t i = 0; i < count; i++) {
    if (bitmask & (1 << bits[i].bit)) {
      RCLCPP_WARN(this->get_logger(), "%s: %s", type, bits[i].description);
    }
  }
}

rcl_interfaces::msg::SetParametersResult Pio36Node::onParameterUpdate(
    const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";
  
  bool dither_changed = false;

  for (const auto &param : params) {
    try {
      const std::string &param_name = param.get_name();

      // Handle dither parameters
      if (param_name == "dither.duty") {
        float new_duty = param.as_double();
        if (new_duty < 0.0 || new_duty > 100.0) {
          result.successful = false;
          result.reason = "dither.duty must be between 0 and 100%";
          RCLCPP_WARN(this->get_logger(), "Rejected dither.duty=%.2f (out of range 0-100%%)", new_duty);
          continue;
        }
        params_.dither.duty = new_duty;
        dither_changed = true;
      }
      else if (param_name == "dither.frequency") {
        float new_freq = param.as_double();
        if (new_freq < 0.0 || new_freq > 250.0) {
          result.successful = false;
          result.reason = "dither.frequency must be between 0 and 250 Hz (Nyquist limit)";
          RCLCPP_WARN(this->get_logger(), "Rejected dither.frequency=%.2f (out of range 0-250Hz)", new_freq);
          continue;
        }
        params_.dither.frequency = new_freq;
        dither_changed = true;
      }
      else if (param_name == "dither.phase") {
        float new_phase = param.as_double();
        if (new_phase < 0.0 || new_phase > 360.0) {
          result.successful = false;
          result.reason = "dither.phase must be between 0 and 360 degrees";
          RCLCPP_WARN(this->get_logger(), "Rejected dither.phase=%.2f (out of range 0-360°)", new_phase);
          continue;
        }
        params_.dither.phase = new_phase;
        dither_changed = true;
      }

    } catch (const std::exception &e) {
      result.successful = false;
      result.reason = std::string("Failed to update parameter: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Parameter update exception: %s", e.what());
    }
  }
  
  // Apply changes to device if connected
  if (connected_ && result.successful) {
    try {
      if (dither_changed) {
        applyDither();
      }
    } catch (const std::runtime_error& err) {
      result.successful = false;
      result.reason = std::string("Failed to write to device: ") + err.what();
      RCLCPP_ERROR(this->get_logger(), "Failed to apply parameter changes to device: %s", err.what());
    }
  }

  return result;
}

NS_FOOT
