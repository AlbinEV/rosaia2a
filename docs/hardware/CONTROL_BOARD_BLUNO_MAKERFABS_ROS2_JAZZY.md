# Control Board Contract — Bluno v2.0 + Makerfabs H-Bridge Motor Shield v1.0

## Purpose

This document is the hardware/software contract for an AI coding agent that will help develop:

- Arduino firmware for the Bluno
- ROS 2 Jazzy software on the host PC
- serial communication between PC and Bluno
- motor control, encoder acquisition and current sensing

Treat the pin mapping and wiring below as fixed unless the user explicitly changes the hardware.

> Important: the first physical motor currently being tested is connected to the **Makerfabs M2 channel**, not M1.

---

## 1. System architecture

```text
ROS 2 Jazzy PC
      |
      | USB serial
      v
DFRobot Bluno v2.0
  ATmega328-class MCU
      |
      | Arduino shield pins
      v
Makerfabs H-Bridge Motor Shield v1.0
      |
      +--------------------+
      |                    |
   channel M1           channel M2
      |                    |
   DC motor             DC motor
      ^                    ^
      |                    |
 encoder + ACS712     encoder + ACS712
```

The PC performs high-level ROS 2 logic.

The Bluno performs low-level deterministic I/O:

- PWM generation
- H-bridge direction control
- encoder counting
- current-sensor ADC sampling
- serial parsing
- telemetry
- communication watchdog
- motor stop on communication loss

Do **not** try to run a full ROS 2 stack directly on the ATmega328. Use a small custom serial protocol between the ROS 2 node and Arduino firmware.

---

## 2. Bluno board

Board:

- DFRobot Bluno / Bluno v2.0
- Arduino UNO compatible
- ATmega328-class MCU
- 5 V logic
- integrated TI CC2540 BLE
- Arduino UNO pin mapping
- use `Serial` for PC communication

Reserve:

```text
D0 = UART RX
D1 = UART TX
```

Do not reuse D0/D1 for encoders or sensors.

For ROS development, prefer USB serial over BLE.

Recommended serial configuration:

```text
115200 baud
8 data bits
no parity
1 stop bit
```

---

## 3. Makerfabs H-Bridge Motor Shield v1.0

Model:

```text
Makerfabs OAS0000HB
```

The shield contains two independent H-bridges.

Official motor supply range:

```text
6–22 V
```

For the current motors, use approximately 6 V.

### Fixed shield pin mapping

| Function | Bluno pin |
|---|---:|
| Global motor enable | `D6` |
| M1 PWM / speed | `D9` |
| M1 direction 1 | `D4` |
| M1 direction 2 | `D5` |
| M2 PWM / speed | `D10` |
| M2 direction 1 | `D7` |
| M2 direction 2 | `D8` |

These pins are already connected through the stacked shield.

Firmware should centralize them:

```cpp
constexpr uint8_t MOTOR_ENABLE_PIN = 6;

constexpr uint8_t M1_PWM_PIN  = 9;
constexpr uint8_t M1_DIR1_PIN = 4;
constexpr uint8_t M1_DIR2_PIN = 5;

constexpr uint8_t M2_PWM_PIN  = 10;
constexpr uint8_t M2_DIR1_PIN = 7;
constexpr uint8_t M2_DIR2_PIN = 8;
```

Avoid scattering literal pin numbers through the firmware.

---

## 4. Current physical wiring

### Physical Motor 1

The first physical motor is connected to the **M2 output** of the Makerfabs shield.

```text
physical_motor_1 -> shield M2
```

Current motor-power wiring:

```text
M2A -------- red motor wire

M2B ---- ACS712 current path ---- black motor wire
```

Equivalent circuit:

```text
M2A -------- RED
              |
          [ DC MOTOR ]
              |
            BLACK
              |
           ACS712
              |
M2B ----------+
```

The ACS712 is in series with the black motor lead.

The exact `IP+` / `IP-` orientation of the ACS712 must be recorded before interpreting current sign.

Reversing `IP+` and `IP-` only reverses the sign of the measured current.

---

## 5. Encoder wiring

The motor encoder has four wires:

| Wire | Function |
|---|---|
| Blue | `+5 V` |
| Green | `GND` |
| Yellow | quadrature channel A |
| White | quadrature channel B |

Encoder ground is shared with ACS712 ground and Bluno ground.

Encoder supply and ACS712 supply both use the Bluno 5 V rail.

Motors themselves are **not** powered from Bluno 5 V.

---

## 6. Current sensors

Use one ACS712 per motor so the two motor currents are measured independently.

### Physical Motor 1

| ACS712 pin | Connection |
|---|---|
| `VCC` | Bluno `5V` |
| `GND` | Bluno/common `GND` |
| `OUT` | `A0` |
| current path | in series with M2B / black motor wire |

### Physical Motor 2

| ACS712 pin | Connection |
|---|---|
| `VCC` | Bluno `5V` |
| `GND` | Bluno/common `GND` |
| `OUT` | `A1` |
| current path | in series with the second motor H-bridge branch |

Do not use a single ACS712 before the common motor power input if independent motor currents are required.

---

## 7. Encoder interrupt strategy

ATmega328 has two dedicated external interrupt pins:

```text
D2 = INT0
D3 = INT1
```

There are only two dedicated external interrupts.

In the present one-primary-motor-per-controller setup, D2 and D3 are the two
quadrature signals for physical motor 1. The firmware interrupts on D2 and
samples D3 to determine direction.

### Verified current allocation

#### Physical Motor 1 — shield M2

```text
Encoder A / yellow -> D2 / INT0
Encoder B / white  -> D3 / INT1
ACS712 OUT         -> A0
```

#### Physical Motor 2 — shield M1

```text
Encoder A / yellow -> D11 / PCINT3
Encoder B / white  -> D12 / PCINT4
ACS712 OUT         -> A1
```

The firmware enables a pin-change interrupt only on D11 and samples D12 in the
ISR, giving x2 decoding without consuming D2/D3 or interfering with Timer1 PWM
on D9/D10.

Recommended initial encoder decoding:

- interrupt on channel A using `CHANGE`
- read channel B inside the ISR
- update signed encoder count
- keep ISR extremely short

Do not do the following inside an encoder ISR:

- `Serial.print`
- floating-point calculations
- `analogRead`
- delays
- filtering

If x4 quadrature decoding is required later, use pin-change interrupts or a suitable lightweight encoder implementation after checking pin/timer conflicts.

---

## 8. Complete proposed pin map

### Pins used by Makerfabs shield

| Pin | Function |
|---|---|
| `D4` | M1 direction 1 |
| `D5` | M1 direction 2 |
| `D6` | motor shield enable |
| `D7` | M2 direction 1 |
| `D8` | M2 direction 2 |
| `D9` | M1 PWM |
| `D10` | M2 PWM |

### Feedback pins

| Pin | Function |
|---|---|
| `D2` | Physical Motor 1 encoder A |
| `D3` | Physical Motor 1 encoder B |
| `A0` | Physical Motor 1 current |
| `A1` | Physical Motor 2 current |
| `D11` | Physical Motor 2 encoder A / PCINT3 |
| `D12` | Physical Motor 2 encoder B / PCINT4 |

### Reserved

| Pin | Reason |
|---|---|
| `D0` | UART RX |
| `D1` | UART TX |

Available unless later used:

```text
D13
A2
A3
A4
A5
```

---

## 9. Naming rule

Do not confuse physical motor numbering with shield channel numbering.

Current mapping:

```text
physical_motor_1 / axis_0 -> shield M2
physical_motor_2 / axis_1 -> shield M1   # planned
```

Firmware hardware-layer names should use:

```text
shield_m1
shield_m2
```

ROS/logical names should use:

```text
axis_0
axis_1
```

or actual joint names once known.

Keep the logical-to-hardware mapping in one place only.

---

## 10. Motor command abstraction

Firmware should expose an abstraction equivalent to:

```cpp
void setMotorCommand(MotorChannel channel, int16_t command);
```

Command range:

```text
-255 ... +255
```

Interpretation:

```text
command > 0  -> one direction
command < 0  -> opposite direction
command == 0 -> stop
PWM = abs(command)
```

Do not assume that positive command corresponds to a meaningful physical robot direction until experimentally verified.

The direction truth table for the Makerfabs shield should be hidden inside the motor-driver layer.

---

## 11. Current measurement

ATmega328 ADC is 10-bit.

Nominal ADC conversion with 5 V reference:

```text
5.0 / 1023 ~= 4.89 mV/count
```

ACS712 output is nominally near half supply at zero current:

```text
Vzero ~= 2.5 V
```

Do not hard-code exactly 2.500 V.

At startup:

1. motors disabled
2. acquire many ADC samples
3. calculate zero-current offset independently for each ACS712
4. store it for later conversion

Current equation:

```text
I = (Vadc - Vzero) / sensitivity
```

Typical ACS712 sensitivities:

| Variant | Approx. sensitivity |
|---|---:|
| 5 A | 185 mV/A |
| 20 A | 100 mV/A |
| 30 A | 66 mV/A |

The exact module variant is currently **unknown** and must remain configurable.

PWM causes pulsating current. Do not use one raw `analogRead()` as the final current estimate.

Use:

- repeated samples
- averaging and/or low-pass filtering
- configurable calibration values

During development, telemetry should expose both raw ADC and filtered current.

---

## 12. Power

Logic/sensor power:

```text
Bluno USB / board supply
Bluno 5 V -> encoder VCC
Bluno 5 V -> ACS712 VCC
common GND
```

Motor power:

```text
external ~6 V supply -> Makerfabs motor power input
```

Never power the DC motors directly from the Bluno 5 V pin.

A 100 uF / 63 V electrolytic capacitor is acceptable as a bulk capacitor on a 6 V motor rail.

Polarity:

```text
capacitor + -> motor supply positive
capacitor - -> GND
```

---

## 13. PWM / timer constraint

The Makerfabs shield uses:

```text
D9  -> M1 PWM
D10 -> M2 PWM
```

On ATmega328P, D9 and D10 use Timer1.

Do not introduce libraries that reconfigure Timer1 without explicitly checking their effect on motor PWM.

Start with ordinary Arduino `analogWrite()`.

---

## 14. Firmware safety requirements

Startup must be fail-safe.

Recommended startup sequence:

```text
1. configure motor pins
2. set PWM = 0
3. set direction pins to safe state
4. keep motor outputs disabled
5. initialize serial
6. calibrate ACS712 zero offsets
7. initialize encoder counters
8. enable motor system only when ready
```

Communication watchdog is mandatory.

Recommended timeout:

```text
200–500 ms
```

If no valid motor command arrives before timeout:

```text
PWM = 0
motor enable = OFF
fault flag = communication timeout
```

Malformed serial packets must never leave a stale non-zero command active indefinitely.

---

## 15. Serial protocol

Initial protocol should be simple ASCII, one packet per line.

### PC -> Bluno

Recommended motor command:

```text
CMD,<seq>,<axis0_cmd>,<axis1_cmd>
```

Example:

```text
CMD,152,80,-40
```

where commands are:

```text
-255 ... +255
```

Current mapping:

```text
axis0 -> physical_motor_1 -> shield M2
axis1 -> physical_motor_2 -> shield M1
```

Useful commands:

```text
PING,<seq>
STOP
ZERO_ENC
CAL_CURRENT
```

Do not mix uncontrolled debug text into the machine-readable serial stream.

---

## 16. Telemetry protocol

Recommended Bluno -> PC packet:

```text
STATE,<seq>,<mcu_us>,<enc0>,<enc1>,<adc0>,<adc1>,<current0_mA>,<current1_mA>,<flags>
```

Example:

```text
STATE,152,38124930,12584,-3402,522,507,410,-120,0
```

Prefer integer telemetry on the microcontroller where practical.

The PC can convert units to floating point.

---

## 17. Firmware loop architecture

```text
encoder interrupts:
    update counts only

main loop:
    parse serial input
    update watchdog
    apply motor commands
    sample current sensors
    filter current
    compute encoder deltas
    optionally estimate velocity
    send telemetry
```

Avoid blocking `delay()` in final firmware.

Use `millis()` / `micros()` scheduling.

Suggested initial rates:

```text
encoder ISR       -> event driven
ROS command       -> 50–100 Hz
telemetry         -> 50–100 Hz
current sampling  -> 100–500 Hz
```

---

## 18. ROS 2 Jazzy architecture

Recommended package:

```text
bluno_motor_bridge
```

The ROS 2 node runs on the PC.

Responsibilities:

- open serial port
- reconnect after USB reset/disconnect
- send motor commands
- parse telemetry
- check packet validity
- expose communication status
- publish encoder/current data
- enforce command limits
- convert encoder counts to physical units once calibration is known

Recommended node:

```text
bluno_motor_bridge_node
```

Potential ROS interfaces:

```text
Subscriptions:
    /motor_command

Publications:
    /encoder_counts
    /motor_current
    /motor_state
    /bluno/status
```

Once encoder resolution and joint mapping are known, publish:

```text
/joint_states
```

using:

```text
sensor_msgs/msg/JointState
```

Long term, consider a `ros2_control` hardware interface.

Do not begin with full `ros2_control` before basic serial control and telemetry are verified.

---

## 19. ROS parameters

Keep calibration and system-specific values configurable.

Example:

```yaml
serial_port: /dev/ttyACM0
baud_rate: 115200

command_rate_hz: 100.0
telemetry_rate_hz: 100.0
command_timeout_ms: 300

axis0:
  shield_channel: M2
  encoder_cpr: null
  gear_ratio: null
  encoder_sign: 1
  motor_sign: 1
  current_sensor:
    analog_pin: A0
    sensitivity_v_per_a: null
    zero_offset: auto

axis1:
  shield_channel: M1
  encoder_cpr: null
  gear_ratio: null
  encoder_sign: 1
  motor_sign: 1
  current_sensor:
    analog_pin: A1
    sensitivity_v_per_a: null
    zero_offset: auto
```

Do not invent unknown values.

---

## 20. Unknowns that must remain configurable

The coding agent must not invent:

- encoder PPR/CPR
- gearbox ratio
- ACS712 version
- exact current sensitivity
- current safety threshold
- physical positive motor direction
- physical positive encoder direction
- exact serial device path
- robot joint names
- maximum safe PWM/current for the mechanism

Ask for measurements or expose parameters instead.

---

## 21. Immediate development milestone

Only Physical Motor 1 is currently required.

It is connected to shield M2.

First firmware milestone:

```text
M2 control
+ encoder 1
+ ACS712 on A0
+ USB serial command
+ telemetry
+ watchdog
```

Do not command M1 yet.

Recommended bring-up sequence:

```text
1. boot with motor disabled
2. calibrate ACS712 zero
3. open serial from PC
4. verify PING/PONG
5. command low positive PWM on M2
6. verify motor motion
7. verify encoder count changes
8. verify current changes
9. STOP
10. command low negative PWM
11. verify encoder direction/sign
12. verify current sign
13. STOP
14. kill/disconnect ROS node
15. verify watchdog automatically stops the motor
```

Only after this is reliable should the second motor be added.

---

## 22. Recommended repository layout

```text
robot_motor_control/
├── README.md
├── hardware/
│   └── CONTROL_BOARD_BLUNO_MAKERFABS.md
├── firmware/
│   └── bluno_motor_controller/
│       ├── bluno_motor_controller.ino
│       ├── motor_driver.h
│       ├── encoder.h
│       ├── current_sensor.h
│       └── serial_protocol.h
└── ros2_ws/
    └── src/
        └── bluno_motor_bridge/
            ├── package.xml
            ├── CMakeLists.txt or setup.py
            ├── config/
            │   └── controller.yaml
            ├── launch/
            │   └── bluno_motor_bridge.launch.py
            └── src/ or bluno_motor_bridge/
```

Keep Arduino firmware independent of ROS internals.

---

## 23. Official references

Makerfabs H-Bridge Motor Shield:

- https://wiki.makerfabs.com/H_Bridge_Motor_Shield.html
- https://www.makerfabs.com/h-bridge-motor-shield.html

DFRobot Bluno DFR0267:

- https://wiki.dfrobot.com/dfr0267/

Bluno serial example:

- https://wiki.dfrobot.com/dfr0267/docs/21923

Prefer official documentation over forum posts when resolving hardware ambiguity.

---

## 24. Current state summary

```text
Controller:
    DFRobot Bluno v2.0 / Arduino UNO-compatible

Motor driver:
    Makerfabs H-Bridge Motor Shield v1.0

Physical Motor 1:
    connected to Makerfabs M2A/M2B

Motor wiring:
    red   -> M2A directly
    black -> M2B through ACS712

Sensor power:
    encoder GND + ACS712 GND -> common GND
    encoder VCC + ACS712 VCC -> Bluno 5 V

Planned feedback for physical_motor_1:
    encoder A/yellow -> D2
    encoder B/white  -> D3
    ACS712 OUT       -> A0

Planned physical_motor_2:
    feedback allocation pending system-architecture confirmation

ROS:
    ROS 2 Jazzy on host PC
    USB serial bridge to Bluno
```

Update this document whenever the physical wiring changes.
