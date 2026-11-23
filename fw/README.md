# Firmware

ESPHome firmware for the valve actuator: a NEMA17 driven by the TMC2209 on a
[LilyGO TTGO T-Motor](https://github.com/Xinyuan-LilyGO/T-Motor). Home
Assistant sees it as a thermostat (a climate entity, with Home, Sleep and Away
presets) and as a valve with a position (0 % closed, 100 % open).

- [`thermostat.yaml`](thermostat.yaml): the device config (Wi-Fi, API, OTA,
  buttons, calibration and its tuning controls).
- [`components/valve_actuator/`](components/valve_actuator/): a custom C++
  ESPHome component. It configures the TMC2209 over UART, generates STEP/DIR
  with an acceleration ramp, finds the end stops by stall detection, and
  remembers the position across reboots. Its climate platform is the
  thermostat.

The PlatformIO bring-up and torque-test firmware lives on the
`archive/torque-test` branch. Why the firmware works the way it does (no limit
switches, encoder stall detection, StealthChop, the power-cut behaviour) is in
[`docs/design-decisions.md`](../docs/design-decisions.md).

## Build and flash

Needs ESPHome (`pip install esphome`); the config was last checked with
2026.11.0.

```sh
cd fw
cp secrets.yaml.example secrets.yaml   # then fill it in
esphome run thermostat.yaml --device /dev/ttyACM0 --upload_speed 115200  # first flash, over the downloader
esphome run thermostat.yaml --device thermostat.local                    # later, over Wi-Fi
```

The first flash needs the downloader's EN and G0 wired to the board, or hold
BOOT, tap RESET, release BOOT. 460800 baud drops the link on this board.

The TMC2209 only answers once the 9 to 12 V motor supply is on. Until then the
valve ignores commands and the log says so; the driver is picked up when it
appears.

Watch the log with `esphome logs thermostat.yaml --device thermostat.local`.

## Home Assistant

Add the device under Settings > Devices & services > ESPHome (it is usually
discovered on its own) and paste `api_encryption_key` from `secrets.yaml` when
asked. It shows up as:

- The thermostat, as the device's main entity (named after the device, e.g.
  "Thermostat"). Off or heat, a target temperature, and the Home, Sleep and
  Away presets. See [Thermostat](#thermostat).
- **Temperature sensor**, **Sleep temperature sensor**, **Away temperature
  sensor**: which Home Assistant sensor the thermostat follows.
- **Valve held by hand**: on while a direct valve command holds off the
  thermostat.
- **Thermostat P gain**, **Thermostat I gain**: tuning.
- **Radiator valve**: a valve with a position, 0 % closed to 100 % open.
- **Calibrate valve**: finds the closed end again by driving into it.
- **Set closed here**, **Set open here**: the valve is at that end right now
  (after turning it there by hand or with the jog buttons).
- **Jog open 10°**, **Jog closed 10°**: move a little, ignoring the 0 to 100 %
  range.
- **Valve needs attention**: raised after the valve recovered from a power cut.
  **Dismiss alert** clears it, and so does a calibration.
- **Stall threshold**, **Calibration speed**, **Calibration current**: tuning,
  see below.
- **StallGuard**, **Valve travel**, **Valve status**: diagnostics.
- **Restart**.

The board's three buttons move the valve to 0, 50 and 100 %.

## Position and the encoder

The T-Motor has an MT6816 magnetic encoder under the motor, reading a magnet on
the back of the motor shaft. The firmware reads it all the time:

- **Turning by hand.** With the motor unpowered between moves, the valve can be
  turned by hand. The position follows, and is published and saved once it has
  been still for 2 s.
- **Lost steps.** After each move the step count is checked against the
  encoder and corrected if they disagree by more than 4 full steps.
- **Stall detection.** While searching for an end stop, the motor falling
  behind its steps by 3 full steps counts as a stall.

The encoder only sees the motor. If a gear slips on its shaft, the motor keeps
turning and the encoder can't tell.

The encoder's direction relative to the motor is learned on the first move of
at least 1/16 motor turn (a jog does it), then saved. Until then hand turns
aren't tracked. If the encoder doesn't follow the motor at all, or reports no
magnet, it's switched off and the log says so; everything else still works
on step counting.

## First-time setup

1. Press **Jog open 10°** once so the encoder learns its direction.
2. Turn the valve by hand (or jog it) to fully closed and press
   **Set closed here**.
3. Turn it to fully open and press **Set open here**. That saves the travel,
   which then wins over `travel` in the YAML.

That's enough to run without ever driving into an end stop. **Calibrate valve**
and the power-cut recovery below find the closed end by stall detection, which
needs tuning first.

## Calibration

When **Calibrate valve** is pressed, the valve:

1. powers the motor at standstill for a moment so StealthChop can tune itself;
2. moves 15° open first if it's near the closed end, so it has room to get up
   to speed;
3. turns towards closed at the calibration speed and current until it stalls,
   and calls that 0 %;
4. backs off 5°, and if the travel isn't known yet (no **Set open here**, no
   `travel` in the YAML), turns towards open until it stalls again and saves
   the distance as the travel;
5. moves back to the position it was asked for.

A stall is either the motor falling behind (encoder) or the driver's
StallGuard load reading dropping three times in a row below **Stall
threshold** of its free-running value. If the valve was already calibrated and
the "end" turns up more than 20° from the old zero, that's treated as a stall
on friction: the old zero is kept and **Valve status** says "Stalled before the
end". If nothing stalls within `max_travel`, the valve assumes it ran into the
closed end anyway and says "Ready, end not detected". Pressing stop on the
valve cancels a calibration.

With these gears the stall has to happen in the motor, before the gear slips
on its shaft: StallGuard doesn't change while the gear slips (measured
2026-11-19), and neither does the encoder. So the calibration runs at a lower
current than normal moves.

**Tuning the calibration current.** Press **Calibrate valve** and watch the log
and **Valve status**:

- "End found at ... (motor stalled)" close to 0°: good.
- "Stalled before the end": the motor can't turn the valve at this current.
  Raise **Calibration current** a step.
- "Ready, end not detected", with the gear slipping at the stop: the motor is
  stronger than the gear's grip. Lower **Calibration current** a step.

**Stall threshold** only matters for StallGuard. If StallGuard trips mid-travel,
lower it. The sliders take effect at the next calibration and survive reboots;
they take over from `speed`, `current` and `stall_threshold` in the YAML's
`homing:` block and start at the same values.

## Power cuts

The position is saved after every move and hand turn, with the encoder angle
at that moment. At boot:

- If the encoder reads within ~5° (motor) of the saved angle, the saved
  position is kept. The encoder only knows the angle within one motor turn, so
  a hand turn of almost exactly whole motor turns while the power was off would
  go unnoticed.
- Without a working encoder, the saved position is kept only after a software
  restart (OTA, **Restart**), not after a power cut.
- Otherwise the valve recovers: it drives closed for up to `max_travel`, calls
  that 0 %, opens to `recovery_position` (5 %) so the radiator can't freeze,
  and raises **Valve needs attention**. Set the valve where you want it and
  press **Dismiss alert**.

## Thermostat

The climate platform sets the valve from the room temperature. It's the
device's main entity (`name: None`), so it takes the device's name and a
second device, say a kitchen radiator with `friendly_name: Kitchen
thermostat`, gets its own climate entity named after it.

**Which sensor.** The temperature comes from a Home Assistant sensor, chosen on
the device page in Home Assistant by typing its entity ID (for example
`sensor.room_meter_bed_temperature`) into a text field:

- **Temperature sensor**: used for Home, a temperature set by hand, and any
  preset without its own sensor. Starts as the Hub 3 in the room.
- **Sleep temperature sensor**: used in the Sleep preset. Starts as the meter
  by the bed.
- **Away temperature sensor**: used in the Away preset. Starts as the meter
  by the window (the coldest spot). Empty means the main sensor.

Any temperature sensor in Home Assistant works, such as a Bluetooth meter or a
"Combine the state of several sensors" helper averaging a few. A change takes
effect at once, without a restart, and is kept across reboots. The starting
values are the substitutions at the top of `thermostat.yaml`.

It's a PI controller, built to move the valve as little as possible:

- It runs once a minute (`update_interval`), and right away when the target,
  preset or mode changes.
- The valve opens 20 % per degree below the target (P gain), plus an integral
  that learns where the valve has to sit to hold the target: it grows 5 % per
  degree-hour below the target (I gain), but only while the room is more than
  0.2 °C (`deadband`) off.
- The valve only moves when the result is at least 5 % (`min_step`) from where
  it is, or to close fully. So noise of a tenth of a degree never moves it.
- The integral is saved, so a reboot or power cut picks up where it was.

**Presets.** Home 23.5 °C, Sleep 20.5 °C and Away 18.5 °C. Choosing Sleep or
Away starts the valve where it usually holds that temperature (25 % and 5 %).
Setting a temperature by hand clears the preset; it holds until something sets
a preset or temperature again (a schedule, for instance).

**Off** closes the valve.

**Manual override.** Moving the valve directly holds it there, and **Valve held
by hand** turns on: the Home Assistant valve entity, the board buttons, the jog
buttons and automations that set the valve all count. The thermostat takes over
again at the next change to its mode, target or preset. So an automation that
pauses heating should turn the thermostat off and back on rather than set the
valve; then a power cut during the pause also keeps it off.

If Home Assistant or the sensor is unavailable, the valve stays where it is.

**Tuning.** The log prints each step at debug level ("Room 20.3, target 20.5:
valve 27% (P +4%, I 23%)"). If the room overshoots after a preset change, lower
the I gain. If it takes hours to settle, raise it. If the valve moves too often,
lower the P gain or raise `min_step`.

## Configuration

All motion settings are in valve degrees, after the gearbox:

| Option | Default | Notes |
| --- | --- | --- |
| `travel` | measured | Valve rotation from closed to open. **Set open here** overrides it. Leave out to have calibration measure it |
| `gear_ratio` | 1.0 | Motor turns per valve turn, e.g. 2.0 for a 1:2 reduction |
| `run_current` | 1.0 A | RMS. Max ~1.64 A with the 120 mOhm sense resistors, but the driver overheats within seconds above ~1.2 A without a heatsink |
| `hold_current` | 30% | Only matters with `hold_when_idle: true` |
| `max_speed` | 90 | Valve °/s. Torque drops with speed |
| `acceleration` | 180 | Valve °/s² |
| `microsteps` | 16 | Interpolated to 256 by the driver |
| `spread_cycle` | true | More torque under load; false = quiet StealthChop (used here). Calibration always runs in StealthChop |
| `hold_when_idle` | false | Off = motor unpowered between moves, so it can be turned by hand |
| `invert_direction` | false | Flip if "open" turns the wrong way. Positive (open) is counter-clockwise on the current motor |
| `address` | scan | TMC2209 UART address from the MS1/MS2 DIP switch (both off = 3) |
| `encoder:` | none | MT6816 on an `spi:` bus: `spi_id` and `cs_pin` |
| `open_limit`, `closed_limit` | none | Optional limit switches (binary sensors). Stop motion in that direction and count as an end stop during calibration |
| `homing:` `on_boot` | true | Recover the position at boot when it can't be trusted |
| `homing:` `speed` | 45 | Valve °/s while searching |
| `homing:` `current` | `run_current` | RMS, while searching |
| `homing:` `stall_threshold` | 50% | StallGuard stall = below this share of its free-running value |
| `homing:` `max_travel` | 1080° | How far to search for an end stop before giving up |
| `homing:` `backoff` | 5° | Move off the closed stop after finding it |
| `homing:` `recovery_position` | 5% | Where to go after recovering from a power cut |
| `alert` | none | Binary sensor raised after a power-cut recovery |
| `stallguard`, `measured_travel`, `status` | none | Diagnostic sensors |

The thermostat (`climate:` `platform: valve_actuator`):

| Option | Default | Notes |
| --- | --- | --- |
| `valve_id` | the valve | |
| `temperature_entity` | none | A text entity (`name`, `initial_value`) holding the Home Assistant sensor to follow. Needs `api:` |
| `sensor` | none | A sensor on the device, used when no Home Assistant sensor is chosen. One of the two is required |
| `humidity_sensor` | none | Shown on the thermostat card |
| `default_target_temperature` | 21 °C | On first boot when there is no `home` preset (otherwise it starts in Home) |
| `presets:` `home`/`sleep`/`away`/`eco`/`comfort`/`boost`/`activity` | none | Each with `target_temperature`, optionally `valve_position` (where the valve usually holds it) and `temperature_entity` (a sensor just for this preset; empty = the main one) |
| `control:` `kp` | 20% | Valve opening per degree below target |
| `control:` `ki` | 5% | Integral growth per degree-hour below target |
| `control:` `deadband` | 0.2 °C | No integral change this close to the target |
| `control:` `min_step` | 5% | Smallest valve move |
| `control:` `max_position` | 100% | Never open the valve further than this |
| `update_interval` | 60 s | |
| `valve_held` | none | Binary sensor: on while a direct valve command holds off the thermostat |

## Board notes

- The SPREAD pin inverts the chopper mode bit, so it's held low and the mode is
  set in software. CLK is held low for the internal clock.
- The VREF trimmer is ignored; current is set over UART only.
- DIAG only drives an LED and doesn't reach the ESP32, so stall detection polls
  `SG_RESULT` over UART while the motor steps, without blocking the step timing.
  StallGuard only works in StealthChop.
- The encoder is on SPI: CLK GPIO14, MOSI GPIO13, MISO GPIO12, CS GPIO15. GPIO12
  and GPIO15 are strapping pins. That's LilyGO's own wiring and the board boots
  normally with it, so the YAML silences ESPHome's warning.
- Limit switches were dropped for the second prototype. The component still
  takes them: P1 has GPIO33 and GPIO32 free (see `hw/limit-switches.md`),
  normally closed, so ON means at the limit or a broken wire.
- Motion runs in ESPHome's main loop. Wi-Fi or API work can delay a step by a
  few ms, which shows as slight unevenness, not lost position.

## Not done yet

- Stall detection during normal moves (only calibration and recovery watch for
  stalls; normal moves only correct lost steps afterwards).
- A temperature sensor on the board, so the thermostat keeps working without
  Home Assistant.
