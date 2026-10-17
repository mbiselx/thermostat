# Firmware

ESP32 firmware for the valve actuator: a NEMA17 driven by the TMC2209 on a
[LilyGO TTGO T-Motor](https://github.com/Xinyuan-LilyGO/T-Motor) board.
Motion uses STEP/DIR; current, microstepping and chopper mode are set over the
TMC2209's UART, so the board's VREF potentiometer has no effect.

## Build and flash

PlatformIO, with the same platform and board settings as LilyGO's reference
project (`espressif32@6.12.0`, `pico32`, Arduino):

```sh
cd fw
pio run -t upload
pio device monitor
```

If upload fails, hold BOOT, tap RESET, release BOOT and try again.

The TMC2209 only answers once the motor supply (JST 6-pin `V+`/`GND`,
9 to 12 V per LilyGO) is connected. USB alone powers the ESP32 but not the
driver; the firmware keeps retrying and configures the driver when it appears.

## Configuration

Defaults live in [`include/config.h`](include/config.h). The ones that matter
for a stiff valve:

| Setting | Default | Notes |
| --- | --- | --- |
| `RUN_CURRENT_MA` | 1000 | RMS. The board tops out at ~1640 mA with its 120 mOhm sense resistors |
| `HOLD_CURRENT_FRACTION` | 0.3 | Only used when `DISABLE_WHEN_IDLE` is false |
| `MAX_SPEED_DEG_S` | 90 | Torque drops with speed, so go slower if it stalls |
| `ACCEL_DEG_S2` | 180 | Gentle ramp to break the valve loose |
| `USE_SPREADCYCLE` | true | More torque under load; StealthChop is quieter |
| `DISABLE_WHEN_IDLE` | true | Motor unpowered between moves; the valve's friction holds it |
| `GEAR_RATIO` | 1.0 | Motor turns per valve turn; angles are in valve degrees |

Pins are in [`include/pins.h`](include/pins.h), taken from LilyGO's README and
schematic.

## Serial console

115200 baud, one command per line:

```text
goto <deg>        move to absolute valve angle
move <deg>        move relative to current angle
zero              call the current angle 0
stop              decelerate and stop
current <mA>      run current, RMS
hold <0-1>        hold current as a fraction of run current
speed <deg/s>     max speed
accel <deg/s^2>   acceleration
mode spread|stealth
idle off|hold     cut power after moves, or keep holding
status
```

Runtime changes are not saved; copy values that work into `config.h`.

### Tuning on the valve

1. Start at `current 800`, `speed 45`, then `move 90` and `move -90`.
2. If the motor buzzes without turning or loses its place, raise the current
   in 100 mA steps, or lower the speed.
3. Watch `status` for overtemperature warnings; the board has no heatsink.

## Board notes

- UART address comes from the MS1/MS2 DIP switch (both off is `0b11`). The
  firmware scans all four unless `DRIVER_ADDRESS` is set.
- The SPREAD pin inverts the chopper mode bit, so the firmware holds it low and
  picks the mode in software. LilyGO's factory example drives it high.
- DIAG only drives an LED, not an ESP32 pin, so stall detection would have to
  poll `SG_RESULT` over UART.
- The MT6816 encoder, OLED header and buttons are wired up in `pins.h` but not
  used yet.
