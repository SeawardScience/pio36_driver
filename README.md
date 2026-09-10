# pio36_driver

**pio36_driver** is a ROS2 package for interfacing with the [Innova PIO36](https://www.innova.no/product/electronic-boards/cute/cute-pio36/) PWM and Analog I/O board over Modbus TCP. The board provides 16 PWM outputs (0-100% duty cycle) for hydraulic valve control and 16 analog voltage inputs (0-10V) for sensor feedback.

Complete API documentation is available at:  [https://seawardscience.github.io/pio36_driver/](https://seawardscience.github.io/pio36_driver/)

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
| `/diagnostics`        | `diagnostic_msgs::msg::DiagnosticArray` | System warnings and sensor alarms, with supply V/A, CPU/Pt100 temperature, alarm limits and water-probe raw ADC |

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
| `dither.duty`                      | `float`       | `0.0`                 | Global dither amplitude (0-100%)                     |
| `dither.frequency`                 | `float`       | `0.0`                 | Global dither frequency (Hz, 0-250)                  |
| `dither.phase`                     | `float`       | `0.0`                 | Global dither phase offset (0-360°)                 |
| `lost_comms_timeout`               | `int`         | `5`                   | Lost communications timeout in seconds               |
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
    
    dither:
      duty: 3.0          # 3% amplitude for hysteresis mitigation
      frequency: 35.0    # 35 Hz (well below 250 Hz Nyquist limit, pwm freq is 500 Hz)
      phase: 0.0         # No phase offset
    
    lost_comms_timeout: 5  # Timeout in seconds before applying lost comms behavior
    deadband: 0.01
```

**Note on Dither Configuration:**
Dither is a global setting applied to all PWM channels on startup to mitigate hydraulic valve hysteresis. Typical values:
- **Duty**: 3-5% amplitude
- **Frequency**: 30-40 Hz (must be < 250 Hz due to 500 Hz PWM carrier)
- **Phase**: 0° (usually not needed)

Adjust based on mechanical testing with your specific valve hardware.

### Runtime Parameter Tuning

Dither parameters support **dynamic reconfiguration** and can be adjusted while the node is running. Changes are validated and immediately written to the hardware.

**Adjust parameters at runtime:**
```bash
# Modify dither amplitude (0-100%)
ros2 param set /pio36 dither.duty 5.0

# Change dither frequency (0-250 Hz)
ros2 param set /pio36 dither.frequency 40.0
```

**View current values:**
```bash
ros2 param get /pio36 dither.duty
ros2 param list /pio36
```

**Parameter validation:**
- `dither.duty`: 0-100% (rejected if out of range)
- `dither.frequency`: 0-250 Hz (Nyquist limit for 500 Hz PWM)
- `dither.phase`: 0-360°

Invalid values are rejected with warning and the device retains its previous setting.

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
| 102            | Holding/Input | System warnings bitmask              |
| 103            | Holding/Input | Sensor alarms bitmask                |
| 104            | Holding/Input | PWM fault status bitmask             |
| 112            | Holding       | Device restart                       |
| 113            | Holding       | Save config to EEPROM                |
| 1033           | Holding/Input | PWM ignore faults bitmask            |
| 1042-1072      | Holding/Input | Dither duty (32-bit float, 0-100%)   |
| 1074-1104      | Holding/Input | Dither frequency (32-bit float, Hz)  |
| 1106-1136      | Holding/Input | Dither phase (32-bit float, 0-360°)  |

### Connection Requirements

- **Network**: Ethernet connection to PIO36 board
- **IP Address**: Default `192.168.52.209` (configurable via parameters)
- **Port**: Standard Modbus TCP port `502`
- **Protocol**: Modbus TCP/IP

---

## Contributing

Contributions are welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to contribute to this project.
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


