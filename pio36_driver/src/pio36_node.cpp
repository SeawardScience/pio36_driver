/** Copyright © 2025 Seaward Science. */

#include "pio36_node.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>

using namespace std::chrono_literals;
using std::placeholders::_1;
NS_HEAD

Pio36Node::Pio36Node()
    : Node("pio36"), diagnostics_updater_(this)
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

  pubs_.valve_state.setup(this, "valve_state");
  pubs_.valve_state.msg_buff.valves.resize(blocks::PwmDuty::NUM_PWM_CHANNELS);

  pubs_.voltage.setup(this, "analog_input_voltage");
  pubs_.voltage.msg_buff.analogs.resize(blocks::AnalogVoltageInput::NUM_CHANNELS);

  diagnostics_updater_.setHardwareID("Innova PIO36 @" + params_.connection.ip);
  diagnostics_updater_.add("System Warnings", this, &Pio36Node::checkSystemWarnings);
  diagnostics_updater_.add("Sensor Alarms",   this, &Pio36Node::checkSensorAlarms);

  // Register parameter update callback for runtime tuning
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Pio36Node::onParameterUpdate, this, std::placeholders::_1));

  // initialize commmands with non zero values so they take effect
  cmd_blocks_.eprom_flush.buffer()[0] = 1;
  cmd_blocks_.restart.buffer()[0] = 1;
  cmd_blocks_.lost_comms_timeout.buffer()[0] = params_.lost_comms_timeout;
  cmd_blocks_.lost_comms_keep_value.buffer()[0] = 0; // 0: set outputs to 0 on timeout
  cmd_blocks_.pwm_ignore_faults.buffer()[0] = computePwmIgnoreFaultsMask();
  cmd_blocks_.sensor_power_config.buffer()[0] =
      params_.sensor_power.ignore_fault ? 0x0001 : 0x0000;  // Bit 0: Ignore sensor power fault (§10.4.13)
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

uint16_t Pio36Node::computePwmIgnoreFaultsMask() {
  uint16_t mask = 0;
  for (auto ch : params_.pwm.ignore_fault_channels) {
    if (ch < 1 || ch > static_cast<int64_t>(blocks::PwmDuty::NUM_PWM_CHANNELS)) {
      RCLCPP_WARN(this->get_logger(),
                  "pwm.ignore_fault_channels: channel %ld out of range (1..%zu), skipping",
                  ch, blocks::PwmDuty::NUM_PWM_CHANNELS);
      continue;
    }
    mask |= static_cast<uint16_t>(1u << (ch - 1));
  }
  return mask;
}

void Pio36Node::applyPwmIgnoreFaults() {
  uint16_t mask = computePwmIgnoreFaultsMask();
  cmd_blocks_.pwm_ignore_faults.buffer()[0] = mask;
  mb_->write_registers(cmd_blocks_.pwm_ignore_faults);
  mb_->read_input_registers(poll_blocks_.pwm_ignore_faults);
  RCLCPP_INFO(this->get_logger(),
              "PWM ignore-faults mask set to 0x%04X (confirmed 0x%04X)",
              mask, poll_blocks_.pwm_ignore_faults.buffer()[0]);
}

uint16_t Pio36Node::sensorAlarmIgnoreMask() const {
  uint16_t mask = 0;
  if (params_.sensor_power.ignore_fault) {
    mask |= (1u << 7);  // §10.5.3.8 Sensor power fault
  }
  return mask;
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
      mb_->read_input_registers(poll_blocks_.onboard_sensors);
      mb_->read_input_registers(poll_blocks_.water_alarm);

      autoClearAlarms();
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

void Pio36Node::addSensorReadings(diagnostic_updater::DiagnosticStatusWrapper & status) {
  auto & s = poll_blocks_.onboard_sensors;
  auto * wa = poll_blocks_.water_alarm.buffer();
  status.addf("Supply voltage (V)", "%.2f", static_cast<double>(s.at(0)));
  status.addf("Supply current (A)", "%.2f", static_cast<double>(s.at(1)));
  status.addf("CPU temperature (C)", "%.1f", static_cast<double>(s.at(2)));
  status.addf("CPU temperature limits (C)", "%.1f / %.1f",
              static_cast<double>(poll_blocks_.cpu_temp_limits.at(0)),
              static_cast<double>(poll_blocks_.cpu_temp_limits.at(1)));
  status.addf("Pt100 temperature (C)", "%.1f", static_cast<double>(s.at(3)));
  status.addf("Water alarm raw ADC", "%u", wa[0]);
  status.addf("Water alarm limits", "%u / %u", wa[5], wa[6]);
}

void Pio36Node::autoClearAlarms() {
  const auto * wa = poll_blocks_.water_alarm.buffer();
  const float cpu = poll_blocks_.onboard_sensors.at(2).get();
  const float cpu_low = poll_blocks_.cpu_temp_limits.at(0).get();
  const float cpu_high = poll_blocks_.cpu_temp_limits.at(1).get();
  const struct { uint8_t bit; bool in_range; } checks[] = {
    {0, wa[0] >= wa[5] && wa[0] <= wa[6]},
    {4, cpu >= cpu_low && cpu <= cpu_high},
  };

  const auto now = std::chrono::steady_clock::now();
  const uint16_t alarms = poll_blocks_.sensor_alarms.buffer()[0];
  // A newly latched bit restarts the hold even if the excursion fell between our samples
  const uint16_t newly_set = alarms & ~last_raw_alarms_;
  last_raw_alarms_ = alarms;
  uint16_t clear_mask = 0;
  for (const auto & c : checks) {
    auto & since = in_range_since_[c.bit];
    if (!c.in_range || (newly_set & (1u << c.bit))) {
      since.reset();
    }
    if (!c.in_range) {
      continue;
    }
    if (!since) {
      since = now;
    }
    if (params_.sensor_alarms.auto_clear_hold_sec > 0.0 && (alarms & (1u << c.bit)) &&
        std::chrono::duration<double>(now - *since).count() >= params_.sensor_alarms.auto_clear_hold_sec) {
      clear_mask |= static_cast<uint16_t>(1u << c.bit);
    }
  }
  if (clear_mask == 0) {
    return;
  }

  cmd_blocks_.sensor_alarms.buffer()[0] = clear_mask;
  mb_->write_registers(cmd_blocks_.sensor_alarms);
  poll_blocks_.sensor_alarms.buffer()[0] = alarms & ~clear_mask;
  for (const auto & b : SENSOR_ALARM_BITS) {
    if (clear_mask & (1u << b.bit)) {
      RCLCPP_INFO(this->get_logger(), "Auto-cleared sensor alarm: %s (reading back in range for %.0fs)",
                  b.description, params_.sensor_alarms.auto_clear_hold_sec);
    }
  }
}

double Pio36Node::inRangeSeconds(uint8_t bit) const {
  const auto & since = in_range_since_[bit];
  if (!since) {
    return -1.0;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - *since).count();
}

void Pio36Node::checkSystemWarnings(diagnostic_updater::DiagnosticStatusWrapper & status) {
  if (!connected_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No data - disconnected");
    return;
  }
  uint16_t warnings = poll_blocks_.system_warnings.buffer()[0];

  if (warnings != prev_warnings_) {
    RCLCPP_INFO(this->get_logger(), "System Warnings changed: 0x%04X -> 0x%04X", prev_warnings_, warnings);
    if (warnings != 0) {
      decodeBitmask(warnings, SYSTEM_WARNING_BITS, std::size(SYSTEM_WARNING_BITS), "System Warning");
    }
    prev_warnings_ = warnings;
  }

  if (warnings == 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No system warnings");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "System warnings detected");
    for (size_t i = 0; i < std::size(SYSTEM_WARNING_BITS); i++) {
      if (warnings & (1 << SYSTEM_WARNING_BITS[i].bit)) {
        status.add(std::string("Bit ") + std::to_string(SYSTEM_WARNING_BITS[i].bit),
                   std::string(SYSTEM_WARNING_BITS[i].description));
      }
    }
  }
}

void Pio36Node::checkSensorAlarms(diagnostic_updater::DiagnosticStatusWrapper & status) {
  if (!connected_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No data - disconnected");
    return;
  }
  uint16_t alarms = poll_blocks_.sensor_alarms.buffer()[0] & ~sensorAlarmIgnoreMask();

  if (alarms != prev_alarms_) {
    RCLCPP_INFO(this->get_logger(), "Sensor Alarms changed: 0x%04X -> 0x%04X", prev_alarms_, alarms);
    if (alarms != 0) {
      decodeBitmask(alarms, SENSOR_ALARM_BITS, std::size(SENSOR_ALARM_BITS), "Sensor Alarm");
    }
    prev_alarms_ = alarms;
  }

  if (alarms == 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No sensor alarms");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Sensor alarms detected");
    for (size_t i = 0; i < std::size(SENSOR_ALARM_BITS); i++) {
      if (alarms & (1 << SENSOR_ALARM_BITS[i].bit)) {
        std::string desc = SENSOR_ALARM_BITS[i].description;
        double in_range = inRangeSeconds(SENSOR_ALARM_BITS[i].bit);
        if (in_range >= 0.0 && params_.sensor_alarms.auto_clear_hold_sec > 0.0) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), " (back in range, auto-clear in %.0fs)",
                        std::max(0.0, params_.sensor_alarms.auto_clear_hold_sec - in_range));
          desc += buf;
        }
        status.add(std::string("Bit ") + std::to_string(SENSOR_ALARM_BITS[i].bit), desc);
      }
    }
  }
  addSensorReadings(status);
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
    mb_->read_input_registers(poll_blocks_.onboard_sensors);
    mb_->read_input_registers(poll_blocks_.water_alarm);
    mb_->read_input_registers(poll_blocks_.cpu_temp_limits);
    RCLCPP_INFO(this->get_logger(),
                "Onboard sensors - supply %.2fV %.2fA, CPU %.1fC (alarm limits %.1f / %.1fC), Pt100 %.1fC",
                static_cast<double>(poll_blocks_.onboard_sensors.at(0)),
                static_cast<double>(poll_blocks_.onboard_sensors.at(1)),
                static_cast<double>(poll_blocks_.onboard_sensors.at(2)),
                static_cast<double>(poll_blocks_.cpu_temp_limits.at(0)),
                static_cast<double>(poll_blocks_.cpu_temp_limits.at(1)),
                static_cast<double>(poll_blocks_.onboard_sensors.at(3)));
    RCLCPP_INFO(this->get_logger(),
                "Water alarm - raw ADC %u, limits %u / %u",
                poll_blocks_.water_alarm.buffer()[0],
                poll_blocks_.water_alarm.buffer()[5],
                poll_blocks_.water_alarm.buffer()[6]);
    uint16_t warnings = poll_blocks_.system_warnings.buffer()[0];
    uint16_t alarms = poll_blocks_.sensor_alarms.buffer()[0] & ~sensorAlarmIgnoreMask();
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

    // Apply sensor-power configuration (bit 0 = ignore fault) from params
    mb_->write_registers(cmd_blocks_.sensor_power_config);
    mb_->read_input_registers(poll_blocks_.sensor_power_config);
    RCLCPP_INFO(this->get_logger(),
                "Sensor power config set to 0x%04X (confirmed 0x%04X)",
                cmd_blocks_.sensor_power_config.buffer()[0],
                poll_blocks_.sensor_power_config.buffer()[0]);

    // Apply per-channel PWM fault-ignore mask from params
    applyPwmIgnoreFaults();

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

  node->declare_parameter<std::vector<int64_t>>(
      "pwm.ignore_fault_channels", pwm.ignore_fault_channels);

  node->declare_parameter("sensor_power.ignore_fault", sensor_power.ignore_fault);
  node->declare_parameter("sensor_alarms.auto_clear_hold_sec", sensor_alarms.auto_clear_hold_sec);

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

  node->get_parameter("pwm.ignore_fault_channels", pwm.ignore_fault_channels);

  node->get_parameter("sensor_power.ignore_fault", sensor_power.ignore_fault);
  node->get_parameter("sensor_alarms.auto_clear_hold_sec", sensor_alarms.auto_clear_hold_sec);

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
