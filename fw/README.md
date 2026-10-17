# Firmware

ESPHome firmware for the valve actuator: a NEMA17 driven by the TMC2209 on a
[LilyGO TTGO T-Motor](https://github.com/Xinyuan-LilyGO/T-Motor). Home
Assistant sees it as a valve with a position (0 % closed, 100 % open).

- [`thermostat.yaml`](thermostat.yaml): the device config (Wi-Fi, API, OTA,
  buttons, limit switches).
- [`components/valve_actuator/`](components/valve_actuator/): a custom C++
  ESPHome component. It configures the TMC2209 over UART, generates STEP/DIR
  with an acceleration ramp, stops at the limit switches, and remembers the
  position across reboots.

The PlatformIO bring-up and torque-test firmware lives on the `torque-test`
branch.

## Build and flash

```sh
cd fw
cp secrets.yaml.example secrets.yaml   # then fill it in
esphome run thermostat.yaml --device /dev/ttyACM0   # first flash, over the downloader
esphome run thermostat.yaml                        # later, over Wi-Fi
```

The first flash needs the downloader's EN and G0 wired to the board, or hold
BOOT, tap RESET, release BOOT.

The TMC2209 only answers once the 9 to 12 V motor supply is on. Until then the
valve ignores commands and the log says so; the driver is picked up when it
appears.

## Configuration

All motion settings are in valve degrees, after the gearbox:

| Option | Default | Notes |
| --- | --- | --- |
| `travel` | required | Valve rotation from closed to open. Measure it |
| `gear_ratio` | 1.0 | Motor turns per valve turn, e.g. 2.0 for a 1:2 reduction |
| `run_current` | 1.0 A | RMS. Max ~1.64 A with the 120 mOhm sense resistors, but the driver overheats within seconds above ~1.2 A without a heatsink |
| `hold_current` | 30% | Only matters with `hold_when_idle: true` |
| `max_speed` | 90 | Valve °/s. Torque drops with speed |
| `acceleration` | 180 | Valve °/s² |
| `microsteps` | 16 | Interpolated to 256 by the driver |
| `spread_cycle` | true | More torque under load; false = quieter StealthChop |
| `hold_when_idle` | false | Off = motor unpowered between moves |
| `invert_direction` | false | Flip if "open" turns the wrong way. Positive (open) is counter-clockwise on the current motor |
| `address` | scan | TMC2209 UART address from the MS1/MS2 DIP switch (both off = 3) |
| `open_limit`, `closed_limit` | none | Binary sensors that stop motion in that direction |

## Board notes

- The SPREAD pin inverts the chopper mode bit, so it's held low and the mode is
  set in software. CLK is held low for the internal clock.
- The VREF trimmer is ignored; current is set over UART only.
- DIAG only drives an LED and doesn't reach the ESP32, so stall detection would
  have to poll `SG_RESULT` over UART (StealthChop only).
- Limit switches on P1 (GPIO33, GPIO32; see `hw/limit-switches.md`) are
  normally closed, so ON means at the limit or a broken wire. They're commented
  out in `thermostat.yaml` until fitted, because an unconnected input reads ON
  and would block all motion.
- Motion runs in ESPHome's main loop. Wi-Fi or API work can delay a step by a
  few ms, which shows as slight unevenness, not lost position.

## Not done yet

- Homing on boot (closed switch first, StallGuard as a fallback). Until then,
  the position is the one saved after the last move.
- Tuning entities in Home Assistant (current, speed).
- The thermostat loop and temperature sensor.
