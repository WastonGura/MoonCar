# MoonCar UART v1

This is the tested software interface, not the vendor sensor protocol. UART is
115200 baud, 8N1, little-endian; wire voltage and pinout must be confirmed before connection.
CRC is integrity checking, not authentication. The link is assumed physically trusted.

| Offset | Field | Size |
| --- | --- | --- |
| 0 | Magic A5 5A | 2 |
| 2 | Version 1 | 1 |
| 3 | Type: command 1, telemetry 2 | 1 |
| 4 | Sequence / latest accepted command sequence | 2 |
| 6 | Payload length, maximum 64 | 2 |
| 8 | Payload | N |
| 8+N | CRC16 CCITT-FALSE of header and payload; init FFFF, poly 1021 | 2 |

Command payload is 5 bytes: int16 linear mm/s, int16 angular mrad/s, uint8 flags.
Flag bit 0 arms, bit 1 latches emergency stop, other bits rejected.
Limits are +/-350 mm/s and +/-800 mrad/s. Host sends at 50 Hz; default is disarmed.
A zero/disarm frame establishes a session; subsequent armed frames need a newer
sequence (modulo 65536). Duplicate armed frames do not refresh the watchdog.

Telemetry payload is 20 bytes: four int32 cumulative encoder counts in FL, FR, RL,
RR order, uint16 battery mV, uint16 fault bits. Fault bit 0 is latched stop; bit 1
is missing command / age >250 ms. Count units are measured counts per output-shaft
revolution after quadrature and gearing, not the encoder marketing resolution.

The bridge converts count differences with int32 wraparound into `/joint_states`.
It requires fresh fault-free telemetry (<500 ms) before arming. A stale host motion
command (>250 ms) sends zero/disarm. Disconnection or an exception terminates the
bridge; MCU watchdog must stop independently. Reconnect requires restarting the
node and explicitly arming again. No automatic reconnect/rearm.

Bench launch (substitute measured values, wheels lifted and motor power initially off):

```bash
ros2 launch mooncar_bringup hardware.launch.py port:=/dev/serial/by-id/YOUR_DEVICE \
  encoder_ticks_per_rev:=MEASURED_COUNTS wheel_radius:=MEASURED_METRES \
  track_width:=MEASURED_EFFECTIVE_METRES
```

Only after the electrical and mechanical checklist passes:

```bash
ros2 service call /mooncar/hardware/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /mooncar/hardware/arm std_srvs/srv/SetBool '{data: false}'
```

This launch is deliberately a motor bench profile. It does not start Gazebo,
Nav2, real sensor drivers, or a guessed physical robot description.
