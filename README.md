# Milk-V Duo AMP F450 Flight Controller

Current architecture is a dual-core AMP flight-control stack:

- `rt_core`: C906 FreeRTOS small-core flight loop for sensor sampling, attitude estimation, PID, PWM, receiver parsing, and failsafe.
- `linux_core`: Linux big-core telemetry/config node for low-rate mailbox doorbells and shared-memory command/config updates.
- `common`: shared protocol and packet definitions for inter-core communication.

## Architecture Mapping

- `Sense`: `rt_core/Sense.*`
- `Receiver`: `rt_core/Receiver.*`
- `Estimator`: `rt_core/Estimator.*`
- `Controller`: `rt_core/Controller.*`
- `Actuator`: `rt_core/Actuator.*`

## AMP Notes

- `Mpu6050I2cBackend` uses RTOS HAL I2C0, checks address conflict with HMC5883L (`0x1E`), and reads MPU6050 at `0x68`.
- `Hmc5883lI2cBackend` uses RTOS HAL I2C0 and reads HMC5883L at `0x1E` for yaw drift correction and diagnostics.
- On Duo 256M wiring, use I2C0 SDA/SCL pins as configured by your board pinout and confirm no address collision before arming.
- `Receiver` parses IBUS from RTOS HAL UART3 at 115200 8N1; channel index `5` over `1500us` triggers immediate failsafe stop.
- Arming requires the real IBUS arm switch high and throttle at or below `1100us`; Linux no longer auto-arms.
- `RtosPwmBackend` maps 4 hardware outputs (`PWM7`,`PWM6`,`PWM10`,`PWM11`) and writes 1000-2000us high time through RTOS HAL PWM.
- Ensure pinmux/device-tree assigns the 4 motor pins to PWM mode before start, otherwise backend falls back and logs a pinmux warning.
- Linux mailbox transport uses `/dev/cvi-rtos-cmdqu` only as a low-rate doorbell/control plane.
- Linux shared-memory setup order:
  - preferred: ION/reserved-memory allocation through the weak board hook `amp_fc_allocate_ion_region()`.
  - debug fallback: `/dev/mem` mapping only when `AMP_CMD_REGION_PHYS=0x...` is explicitly set.
  - host fallback: POSIX shared file (`/dev/shm/amp_fc_cmd_region.bin`) for local simulation only, not RTOS sharing.
- At startup Linux sends `InitSharedMemory` (`cmd=0x20`) once with `param_ptr=physical_address`, then waits for ACK.
- High-rate `Setpoint` and `Heartbeat` update shared memory only; they do not call cmdqu ioctl.
- Linux then notifies RT with blocking mailbox ioctl only for `Arm`, `EmergencyStop`, and `Config`.
- Cache coherency: Linux keeps release fences and exposes `flushSharedRegion()`; RTOS must invalidate/flush DCache around shared-region reads, or run this region uncached.
- RT reads shared command slots in priority order: `EmergencyStop` > `Arm` > `Setpoint` > `Heartbeat` > `Config`.
- RT command polling runs at 1kHz; emergency-stop commands are handled before control update to keep mailbox stop latency within 1ms budget.
- Estimator implements a 2-state Kalman update per axis (angle + gyro bias); Controller implements cascaded PID (angle loop -> rate loop) with integrator clamp anti-windup.
- Camera/RTSP is intentionally moved out of this process and should be run as a separate vendor tool
  (for example `camera_test`, `sample_venc`, or an SDK RTSP sample) to isolate multimedia failures from flight telemetry.

### Dynamic FlightConfig Hot Update

- Shared config structure: `common/protocol.hpp::FlightConfig`
  - `angle_p`, `rate_p`, `rate_i`, `rate_d`, `throttle_expo`, `deadzone_us`, `update_seq`.
- Linux side (`linux_core/TelemetryNode.*`):
  - Sends boot defaults once at startup.
  - Supports runtime CLI on stdin:
    - `set angle_p 5.5`
    - `set rate_p 0.13`
    - `set rate_i 0.02`
    - `set rate_d 0.003`
    - `set throttle_expo 0.45`
    - `set deadzone 15`
    - `show`, `help`
  - Supports `.ini` hot-reload by file-content change.
- RT side (`rt_core/FlightRtNode.*`):
  - Checks mailbox each frame; when `Config` arrives, applies immediately to:
    - `Controller::applyConfig()` for cascaded PID gains.
    - `Receiver::applyConfig()` for deadzone/expo.
  - No reboot required.

CLI/INI run examples:

```bash
./amp_fc/build/linux_telemetry ./amp_fc/flight_config.ini
# or
AMP_FLIGHT_CONFIG_INI=./amp_fc/flight_config.ini ./amp_fc/build/linux_telemetry
```

### FreeRTOS ISR Hook

The RT transport exports two C symbols for direct SDK callback wiring:

- `amp_fc_on_cmdqu_message(unsigned char cmd_id, unsigned int seq_hint)`
- `amp_fc_mailbox_isr(int irq, void* context)`

In your RTOS mailbox callback, call `amp_fc_on_cmdqu_message(cmd_id, param_ptr)` immediately from ISR context.

### Sensor Diagnostics

Small-core C entry points for board bring-up:

- `mpu6050_probe_task(void*)`: prints MPU6050 accelerometer/gyro data.
- `hmc5883l_probe_task(void*)`: prints HMC5883L magnetometer data.
- `sensor_probe_task(void*)`: prints both sensors together.
- `esp8266_probe_task(void*)`: sends basic AT commands to ESP8266 on UART2 at 115200 and prints responses.

Use these tasks before `flight_control_task(void*)` when only one sensor is connected.

### SG2002 HAL Port

- Default `hal_board_*` symbols in `rt_core/hal_impl.cpp` are weak fail-safe stubs.
- `rt_core/Sg2002CsiBoardPort.cpp` adapts the confirmed Duo RTOS I2C API:
  `i2c_init()`, `i2c_write()`, `i2c_read()`, plus `csi_coret_get_value()` for `hal_micros()`.
- Enable it with `AMP_USE_SG2002_CSI_PORT=ON` after placing this project inside the Duo SDK build include path.
- UART init still uses the SDK baud/clock setup, while UART2/UART3 runtime RX/TX uses direct DesignWare MMIO polling to avoid the SDK singleton UART pointer; ESP8266 diagnostics use UART2 by default.

## Duo256M Pinmux Script

Use script:

```bash
sh amp_fc/scripts/duo256m_pinmux_f450.sh uart3
```

- `uart3` avoids GP12/GP13 debug UART conflict.
- Alternative profiles: `uart0`, `uart1`.
