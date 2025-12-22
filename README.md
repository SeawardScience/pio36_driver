# pio36_driver

**pio36_driver** is a ROS2 package for interfacing with the [Innova PIO36](https://www.innova.no/product/electronic-boards/cute/cute-pio36/) PWM and Analog I/O board over Modbus TCP. The board provides 36 PWM outputs (0-100% duty cycle) for hydraulic valve control and 8 analog voltage inputs (0-10V) for sensor feedback.

Complete API documentation is available at:  
👉 [https://seawardscience.github.io/pio36_driver/](https://seawardscience.github.io/pio36_driver/)

---

## Installation

To install `pio36_driver`:

```bash
# Clone the repository into your workspace src/
cd ~/ros2_ws/src
git clone <repository_url>

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### System Dependencies

The driver requires `libmodbus` which is not available through rosdep:

```bash
sudo apt-get install libmodbus-dev
```

The driver also utilizes `io_interfaces` and `hydraulic_interfaces` message types.

### Compiling

To build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select pio36_driver
source install/setup.bash
```

For systems with limited RAM, use sequential build to avoid freezes:

```bash
MAKEFLAGS='-j2' colcon build --executor sequential --packages-select pio36_driver
```

---

## Running the Package

After sourcing your workspace, you can start the PIO36 driver node:

```bash
ros2 run pio36_driver pio36
```

Or use the provided launch file with the example configuration:

```bash
ros2 launch pio36_driver pio36.launch.py
```

To specify a custom configuration file:

```bash
ros2 launch pio36_driver pio36.launch.py config:=/path/to/custom.yaml
```

---

## Nodes

### pio36 Node

The main driver node that communicates with the PIO36 board via Modbus TCP, converting ROS hydraulic valve commands to PWM duty cycles and publishing analog sensor voltages.

#### Published Topics

| Topic                  | Message Type                          | Description                         |
|------------------------|---------------------------------------|-------------------------------------|
| `~/valve_state`        | `hydraulic_interfaces::msg::ValvePack`| PWM duty cycle feedback (0-1 range)|
| `~/analog_input_voltage`| `io_interfaces::msg::RawAnalogArray` | Analog input voltages (0-10V)      |

#### Subscribed Topics

| Topic                  | Message Type                          | Description                         |
|------------------------|---------------------------------------|-------------------------------------|
| `/vessel_ns/control/valve` (configurable) | `hydraulic_interfaces::msg::ValveStamped` | Single valve command (0-1 setpoint) |
| `cmd/valve_pack`       | `hydraulic_interfaces::msg::ValvePack`| Multi-valve command batch          |

#### Parameters

| Parameter Name                     | Type          | Default Value         | Description                                          |
|------------------------------------|---------------|-----------------------|------------------------------------------------------|
| `connection.type`                  | `string`      | `"tcp"`               | Connection type (tcp only)                           |
| `connection.ip`                    | `string`      | `"192.168.52.209"`    | Device IP address                                    |
| `connection.network_port`          | `int`         | `502`                 | Modbus TCP port                                      |
| `timers.poll_ms`                   | `int`         | `500`                 | Input register polling interval (ms)                 |
| `timers.write_ms`                  | `int`         | `10`                  | Output register write interval (ms)                  |
| `timers.connect_ms`                | `int`         | `1000`                | Connection retry interval (ms)                       |
| `topics.valve_cmd`                 | `string`      | `"/vessel_ns/control/valve"` | Single valve command topic               |
| `topics.valve_pack_cmd`            | `string`      | `"cmd/valve_pack"`    | Multi-valve command topic                            |
| `topics.valve_pack_response`       | `string`      | `"resp/valve_pack"`   | Valve state response topic                           |
| `deadband`                         | `float`       | `0.01`                | Minimum setpoint (0-1) before zeroing output         |

Example configuration:

```yaml
/pio36:
  ros__parameters:
    connection:
      type: tcp
      ip: 192.168.52.209
      network_port: 502
    
    timers:
      poll_ms: 500
      write_ms: 10
      connect_ms: 1000
    
    topics:
      valve_cmd: /vessel_ns/control/valve
      valve_pack_cmd: cmd/valve_pack
      valve_pack_response: resp/valve_pack
    
    deadband: 0.01
```

---

## Hardware Interface

Complete hardware documentation: [Innova PIO36 Manual](@ref manual)

### Register Map

The PIO36 board uses Modbus TCP with the following register layout:

| Register Range | Type          | Description                          |
|----------------|---------------|--------------------------------------|
| 1-36           | Holding/Input | PWM duty cycle (32-bit float, 0-100%)|
| 37-44          | Input         | Analog voltage inputs (32-bit float) |
| 45-56          | Holding       | IP configuration                     |
| 65             | Holding/Input | Lost comms timeout (seconds)         |
| 66             | Holding/Input | Lost comms keep value (0/1)          |
| 112            | Holding       | Device restart                       |
| 113            | Holding       | Save config to EEPROM                |

### Connection Requirements

- **Network**: Ethernet connection to PIO36 board
- **IP Address**: Default `192.168.52.209` (configurable via parameters)
- **Port**: Standard Modbus TCP port `502`
- **Protocol**: Modbus TCP/IP

---

## Contributing

Contributions are welcome! When contributing:

- Follow ROS2 coding standards and naming conventions
- Use `ament_cpplint` to check code style (120 character line length)
- Add Doxygen comments for new classes and functions
- Test with actual hardware when possible

```bash
# Check code style
cd ~/ros2_ws/src/pio36_driver/pio36_driver
ament_cpplint src/ include/ --linelength 120
```

---

## Generating Documentation

Documentation is auto-generated using Doxygen:

```bash
cd ~/ros2_ws/src/pio36_driver/docs
doxygen Doxyfile
```

Open the documentation in your browser:

```bash
xdg-open ~/ros2_ws/src/pio36_driver/docs/html/index.html
```

---

## License

This project is licensed under the [LICENSE](LICENSE) file in this repository.

---

## Credits

Developed by [Seaward Science](https://seaward.science/) for ROV control systems.

## Authors
- Dr. Kristopher Krasnosky (lead software engineer)
- Jake Bonney (software developer)


