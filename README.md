# BalancingRobot-SimpleFOC-Motor-Controller

Motor driver firmware of a two-wheeled self-balancing robot (engineering thesis project).
Runs on the **ESP32** of an **MKS DualFOC v3.2 plus** board and drives both BLDC wheel motors with
[SimpleFOC](https://simplefoc.com/) (field-oriented control, current mode). It takes current
commands from the STM32 main controller over UART and sends back wheel speed, position and current.

Part of the Balancing Robot project:

| Repository | Role |
| --- | --- |
| [BalancingRobot-Main-Controller](https://github.com/GrzegorzGrudzinski/BalancingRobot-Main-Controller) | STM32 firmware: IMU, control cascade, state machine, telemetry |
| **BalancingRobot-SimpleFOC-Motor-Controller** (this repo) | ESP32 + SimpleFOC motor driver |
| [BalancingRobot-simulatior](https://github.com/GrzegorzGrudzinski/BalancingRobot-simulatior) | PyBullet simulator |
| [BalancingRobot-DataAnalyser](https://github.com/GrzegorzGrudzinski/BalancingRobot-DataAnalyser) | PC telemetry parser, monitor and logger |

## Hardware

| Part | Details |
| --- | --- |
| Driver board | MKS DualFOC v3.2 plus (ESP32) |
| Motors | 2× BLDC 5010 360 kV, 7 pole pairs |
| Encoders | 2× AS5047P magnetic encoder, SPI (HSPI, CS: GPIO 5 and 15) |
| Link to the STM32 | UART2, RX GPIO 16, TX GPIO 17, 460800 baud |
| USB | serial console, 115200 baud (debugging and test commands) |

All pins and limits are in `motor_driver/config.h` and `motor_driver/comm.h`.

## How it works

- **State machine:** `STANDBY → CONNECT → INIT → RUN`, plus `STOP`, `ERROR` and `MOTOR_TEST`.
  The STM32 starts the handshake; motors are driven only in `RUN` and `MOTOR_TEST`;
  `STANDBY`, `STOP` and `ERROR` disable them.
- **Protocol (STM32 → ESP32):** 12 B frame, start byte `0xAA`: command, sequence number, two
  float values (current targets for the wheels), XOR checksum.
- **Protocol (ESP32 → STM32):** 32 B frame, start byte `0xBB`: status, error, rx/tx sequence
  numbers, velocity, position and current of both motors, XOR checksum.
  Both structs must match the STM32 side (`motor_frames.h` there); `static_assert` checks the size.
- **Protections:** communication timeout (50 ms), stall detection, wheel desync and opposite-spin
  detection; errors are latched until the STM32 sends `CLR_FLT`.
- **Wheel sync:** a small correction current keeps the wheels together when driving straight,
  switched off while turning.
- **Friction compensation** (`src/friction/`): adds a feed-forward current against Coulomb friction;
  on by default (`FRIC_COMP_DEFAULT`), toggled over USB with `F0` / `F1`.

## Files

```
motor_driver/
  motor_driver.ino         setup(), loop(), state machine
  comm.cpp / comm.h        UART protocol to the STM32, USB console commands
  motor.cpp / motor.h      SimpleFOC motors, drivers and sensors, sync, protections
  config.h                 pins, motor parameters, limits, timeouts
  src/friction/            friction compensation (plain C)
```

## Building and flashing

Arduino project (sketch folder `motor_driver/`):

1. Install the ESP32 board package (Espressif) and the **Simple FOC** library in the Arduino IDE.
2. Open `motor_driver/motor_driver.ino`, select an ESP32 board ("ESP32 Dev Module")
   and the USB port, then upload.

## USB console

Open a serial monitor at 115200 baud. Commands (SimpleFOC `Commander`):

| Command | Action |
| --- | --- |
| `I` | init / start FOC |
| `X` | stop motors |
| `S` | toggle motor test mode |
| `T<value>` | target current in test mode |
| `D` | toggle debug telemetry |
| `F0` / `F1` | friction compensation off / on |

## License

MIT, see [LICENSE](LICENSE).
