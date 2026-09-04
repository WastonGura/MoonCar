# STM32 Firmware

`core/mooncar_core.c` is a portable C11 motor-control and UART protocol core.
It has no board-specific HAL, pin assignment, startup code, linker script or flash image.
`tests/emulator.c` hosts the same core on a Linux pseudo-terminal with a toy wheel model.

Run `../../scripts/check_software.sh` from this directory on the ROS Ubuntu host.
This compiles the core with warnings-as-errors and undefined-behavior checks, and
exercises the Python host against the C core over a virtual UART.

Integration contract:

- Call `mc_init` with measured wheel radius/effective track and initially conservative PID gains.
- Feed each UART RX byte into `mc_receive` with a monotonic millisecond counter.
- Call `mc_step` at a measured fixed period (nominal 10 ms); supply signed wheel rad/s,
  hardware emergency-stop input and receive four signed normalized PWM outputs.
- Wheel order is front-left, front-right, rear-left, rear-right. Positive means forward.
- Convert normalized outputs to the chosen driver's PWM/direction/dead-time signals.
- Transmit `mc_telemetry` at 50 Hz using signed cumulative output-shaft encoder ticks.
- Keep UART parsing and control access serialized; do not mutate `mc_core` concurrently
  from DMA/interrupt and task contexts without a board-specific synchronization strategy.
- Configure an independent MCU watchdog and default-low motor enable in the HAL.
  A stopped control task cannot execute the portable software watchdog.
- `external_stop` is latched until reinitialization. A physical stop must independently
  cut the driver enable/power; software alone is not an emergency-stop circuit.

The serial watchdog forces zero PWM after 250 ms without a new valid command.
CRC, bad lengths, duplicates and invalid speed requests cannot refresh an armed command.
No motor tuning or electrical safety has been validated without hardware.
See [hardware handoff](../../docs/hardware_handoff.md) and
[UART protocol](../../docs/uart_protocol.md).
