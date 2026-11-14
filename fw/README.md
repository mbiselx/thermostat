# Firmware

ESPHome firmware for the valve actuator: a NEMA17 driven by the TMC2209 on a
[LilyGO TTGO T-Motor](https://github.com/Xinyuan-LilyGO/T-Motor). Home
Assistant sees it as a valve with a position (0 % closed, 100 % open).

- [`thermostat.yaml`](thermostat.yaml): the device config (Wi-Fi, API, OTA,
  buttons, calibration and its tuning controls).
- [`components/valve_actuator/`](components/valve_actuator/): a custom C++
  ESPHome component. It configures the TMC2209 over UART, generates STEP/DIR
  with an acceleration ramp, finds the end stops by stall detection, and
  remembers the position across reboots.

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

Watch the log with `esphome logs thermostat.yaml`.

## Home Assistant

Add the device under Settings > Devices & services > ESPHome (it is usually
discovered on its own) and paste `api_encryption_key` from `secrets.yaml` when
asked. It shows up as:

- **Radiator valve**: a valve with a position, 0 % closed to 100 % open.
- **Calibrate valve**: finds the end stops again.
- **Stall threshold** and **Calibration speed**: tuning, see below.
- **StallGuard**, **Valve travel**, **Valve status**: diagnostics.
- **Restart**.

## Calibration

There are no limit switches. At boot, and when **Calibrate valve** is pressed,
the valve:

1. switches the driver to StealthChop, the only mode with stall detection;
2. moves 15° open first if it thinks it's near the closed end, so it has room
   to get up to speed;
3. turns towards closed until it stalls, and calls that 0 %;
4. backs off 5°, then turns towards open until it stalls again and saves the
   distance as the valve travel (skipped when `travel` is set in the YAML);
5. goes back to SpreadCycle and moves to the position it was asked for.

Stall detection reads the driver's StallGuard value, which drops as the load
rises. The value while turning freely is measured first, and a stall is three
readings in a row below `stall_threshold` of it. If no stall shows up within
`max_travel`, the valve assumes it has run into the closed end anyway (the
motor slips at the stop) and **Valve status** says so. Pressing stop on the
valve cancels a calibration.

**Tuning on a new valve.** Press **Calibrate valve** and watch **StallGuard**
and the log. You want a clearly higher value while turning freely than at the
stops.

- Stops too early (on friction mid-travel): lower **Stall threshold**.
- Grinds at the end stop: raise **Stall threshold**.
- StallGuard near 0 even when turning freely: raise **Calibration speed** or
  lower `homing: current`. StallGuard gets unreliable at low motor speeds.

The sliders take effect at the next calibration and survive reboots. They take
over from `speed` and `stall_threshold` in the YAML's `homing:` block, and start
at the same values.

## Configuration

All motion settings are in valve degrees, after the gearbox:

| Option | Default | Notes |
| --- | --- | --- |
| `travel` | measured | Valve rotation from closed to open. Leave out to have calibration measure it |
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
| `open_limit`, `closed_limit` | none | Optional limit switches (binary sensors). Stop motion in that direction and count as an end stop during calibration |
| `homing:` `on_boot` | true | Calibrate at boot |
| `homing:` `speed` | 45 | Valve °/s while searching |
| `homing:` `current` | `run_current` | RMS, during calibration |
| `homing:` `stall_threshold` | 50% | Stall = StallGuard below this share of its free-running value |
| `homing:` `max_travel` | 1080° | How far to search for an end stop before giving up |
| `homing:` `backoff` | 5° | Move off the closed stop after finding it |
| `stallguard`, `measured_travel`, `status` | none | Diagnostic sensors |

## Board notes

- The SPREAD pin inverts the chopper mode bit, so it's held low and the mode is
  set in software. CLK is held low for the internal clock.
- The VREF trimmer is ignored; current is set over UART only.
- DIAG only drives an LED and doesn't reach the ESP32, so stall detection polls
  `SG_RESULT` over UART while the motor steps, without blocking the step timing.
- Limit switches were dropped for the second prototype. The component still
  takes them: P1 has GPIO33 and GPIO32 free (see `hw/limit-switches.md`),
  normally closed, so ON means at the limit or a broken wire.
- Motion runs in ESPHome's main loop. Wi-Fi or API work can delay a step by a
  few ms, which shows as slight unevenness, not lost position.

## Not done yet

- Stall detection during normal moves (only calibration watches for stalls).
- The thermostat loop and temperature sensor.
