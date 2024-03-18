#include "pio36_node.hpp"

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
  connect();

  subs_.valve_ = this->create_subscription<hydraulic_interfaces::msg::ValveStamped>(
      params_.topics.valve_cmd, 24, std::bind(&Pio36Node::valveCallback, this, _1));

  subs_.valve_pack_ = this->create_subscription<hydraulic_interfaces::msg::ValvePack>(
      params_.topics.valve_pack_cmd, 1, std::bind(&Pio36Node::valvePackCallback, this, _1));

  write_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.write_ms), std::bind(&Pio36Node::writeBuffer, this));
}

void Pio36Node::valveHandler(hydraulic_interfaces::msg::Valve & valve){
  cmd_blocks_.pwm_duty.dutyChan(valve.valve_id) = valve.set_point * 100.0;
}

void Pio36Node::valveCallback(const hydraulic_interfaces::msg::ValveStamped::SharedPtr msg) {
  valveHandler(msg->valve);
}

void Pio36Node::valvePackCallback(const hydraulic_interfaces::msg::ValvePack::SharedPtr msg){
  for(auto valve: msg->valves){
    valveHandler(valve);
  }
}

void Pio36Node::writeBuffer(){
  RCLCPP_DEBUG(this->get_logger(), "pwm_duty: %s", cmd_blocks_.pwm_duty.hexString().c_str());

  if(connected_){
    try{
      mb_.write_registers(cmd_blocks_.pwm_duty);
    }catch(std::runtime_error err){
      std::string error_msg = err.what();
      RCLCPP_WARN(this->get_logger(),"modbus write failed: ", error_msg.c_str());
    }
  }
}

void Pio36Node::poll(){
  if(connected_){
    try{
      mb_.read_input_registers(poll_blocks_.pwm_duty);
    }catch(std::runtime_error err){
        std::string error_msg = err.what();
        RCLCPP_WARN(this->get_logger(),"modbus read failed: ", error_msg.c_str());
      }
  }
}

void Pio36Node::connect()
{
  if(connected_){
    return;
  }
  try{
    if(params_.connection.type == "tcp"){
      RCLCPP_INFO_ONCE(this->get_logger(),"Connecting to %s:%i",params_.connection.ip.c_str(), params_.connection.network_port);
      mb_.connect_tcp(params_.connection.ip.c_str(), params_.connection.network_port);
    }else{
      RCLCPP_ERROR(this->get_logger(),"Invalid connection type specified %s",params_.connection.type.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),"Connectd to device");
    connect_timer_->reset();
    connected_ = true;
    RCLCPP_INFO(this->get_logger(),"Connected!");
  }catch(std::runtime_error err){
    connected_ = false;
    std::string error_msg = err.what();
    RCLCPP_WARN_ONCE(this->get_logger(),"Unable to connect, will continue trying. Got Error\n ---%s", error_msg.c_str());
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
}

NS_FOOT
