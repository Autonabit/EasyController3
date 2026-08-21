# EasyController Firmware Changelog

## Version 7

### Changes

- **Added board identification.** Each driver now reports the RP2040's
  unique factory-programmed ID over I2C, making it easier to tell
  individual boards apart when several are wired to the same bus.
- **Further throttle refinements.** The throttle slew rate is now a single
  configurable constant, the slewed value is clamped to the valid ±255
  range, and braking immediately zeroes the target throttle instead of
  slewing down to it.
- Reverted PWM frequency back to 2kHz (previously increased to 4kHz in an
  interim build).
- Re-enabled USB debug output by default (previously disabled).
- Minor internal cleanup: removed unused `MIN`/`MAX` macros.

## Version 6

**Action required if your motor driver uses the older 20A current sensor:**
before flashing, set `CURRENT_SENSOR_30A` to `false` near the top of
`easycontroller.c`. All new motor drivers ship with the 30A sensor, so no
change is needed for boards built recently.

### Changes

- **Added support for the 30A current sensor.** Firmware previously only
  supported the 20A sensor's scaling. Both are now supported via a single
  setting (`CURRENT_SENSOR_30A`), which now defaults to the 30A sensor since
  that's what all new motor drivers use. Flashing with the wrong setting for
  the sensor actually fitted will cause incorrect current readings and
  incorrect current limiting.
- **Smoother, more predictable throttle response.** Throttle now ramps to
  the commanded value at a fixed rate rather than speeding up or slowing
  down depending on how busy the controller was at the time. The motor also
  now comes to a firm stop at zero throttle instead of slowly drifting
  toward it.
- **Safer power-up.** Fixed a rare startup condition where the motor gates
  could briefly be driven into a hard-brake state for a moment before the
  controller finished initializing.
- **More reliable I2C status reads.** Fixed a timing issue where a status
  read over I2C could occasionally return a mix of old and new data if it
  landed at the same moment the controller updated its internal state.
- Minor internal cleanup; no behavior change for customers.

## Version 5

- **Fixed a pin conflict that could disrupt I2C communication.** A debug
  "flag" pin shared the same physical pin as the I2C data line (SDA). It has
  been removed, resolving a conflict that could interfere with I2C on any
  board using it.
- **More robust I2C communication.** Reads to an invalid/out-of-range
  address now return `0x00` instead of stale or undefined data. Writes to an
  invalid/out-of-range address are now safely ignored instead of risking
  memory corruption or a crash.
- **Added a watchdog on the main control loop.** If the loop ever stalls for
  more than 10ms, the controller now resets itself automatically rather than
  leaving the motor stuck in whatever state it was last in.
- **Faster response to lost communication.** If the controlling computer
  stops talking to the driver, it now resets after 10 seconds instead of up
  to 10 minutes.
- **Added over-temperature protection on the driver itself.** Throttle is
  now cut to zero if the driver's own temperature sensor exceeds 80°C,
  instead of relying solely on the upstream controller to catch overheating.
- Reduced debug/USB serial output by default during normal operation.

## Version 4

- **Added temperature sensor support.** Motor driver temperature is now
  measured and reported over I2C.
- **Improved current limiting.** Current is now filtered for a more stable
  reading, and the limiter backs off gradually above the current limit and
  recovers gradually once current drops back down, rather than reacting in
  a single abrupt step.
- **Improved speed estimate.** The speed value reported over I2C is more
  robust to noisy or stale sensor data, and decays smoothly to zero when the
  motor stops instead of holding a stale reading.
- **Smoother throttle response.** Throttle changes are now ramped
  ("slewed") rather than applied instantly, most noticeable as smoother
  operation around zero speed.
- **Added a startup LED blink pattern** indicating the driver's I2C address,
  to help identify individual boards when several are wired to the same bus.
- Hall auto-identification on boot is now disabled by default for deployed
  units (previously the motor would briefly self-spin at power-up to learn
  its hall sensor table).
