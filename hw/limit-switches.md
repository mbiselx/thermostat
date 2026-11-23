# Limit switches

> **Not used.** Limit switches were dropped for the second prototype
> (2026-11-14). The firmware finds the closed end by stalling the motor at a
> low current and seeing it with the encoder instead; see
> [`../docs/design-decisions.md`](../docs/design-decisions.md#finding-the-end-stops).
> The firmware still accepts switches (`open_limit`, `closed_limit`), so this
> note is kept for anyone who wants to add them back. The P1 pinout below is
> also the reference for powering the board.

The valve actuator uses up to two limit switches wired to the spare inputs on the
T-Motor's 6-pin power connector (P1). No extra components are needed.

Source: LilyGO's [T-Motor schematic](https://github.com/Xinyuan-LilyGO/T-Motor/blob/main/Schematic/T-Motor.pdf)
and [pin diagram](https://github.com/Xinyuan-LilyGO/T-Motor/blob/main/image/image1.jpg).

## P1 pinout

| P1 pin | Label | ESP32 | Use in this project |
| ------ | ----- | ----- | ------------------- |
| 1 | V+  | -      | Power in (5-12 V, through an SS24 diode to the motor supply) |
| 2 | GND | -      | Power ground, and common ground for the switches |
| 3 | COM | -      | Leave unconnected (only used on the optocoupler build) |
| 4 | EN  | GPIO33 | Limit switch 1 |
| 5 | STP | GPIO25 | Spare (third switch if ever needed) |
| 6 | DIR | GPIO32 | Limit switch 2 |

Despite their names, P1's EN/STP/DIR are not connected to the TMC2209. The driver
is controlled from GPIO2 (EN), GPIO23 (STEP) and GPIO18 (DIR) on the board itself,
so using these pins as inputs does not affect the motor.

GPIO25/32/33 are ordinary bidirectional GPIOs. They are not strapping pins, so a
switch that is closed at power-up does not affect boot. They are RTC GPIOs, so they
can wake the ESP32 from deep sleep if needed.

## Input circuit (MOSFET build)

The board has two assembly options between P1 and the ESP32. Ours is the
**MOSFET build** (three BSS123W transistors next to P1, checked on the board).

```text
            3V3
             |
            2k (RL1..RL3, on board)
             |
  GPIO ------+------ S  BSS123W  D ------ P1 pin (EN / STP / DIR)
                        G = 3V3                  |
                                               switch
                                                 |
                                              P1 GND
```

- Switch open: the 2k pull-up holds the GPIO high.
- Switch closed: the P1 pin is pulled to GND and the MOSFET (or its body diode)
  pulls the GPIO low. The current is about 1.6 mA.

So each input is **active-low**: LOW means the switch is closed.

## Wiring

- Wire each switch between its P1 signal pin and P1 GND.
- Use **normally-closed (NC)** contacts. In normal travel the input reads LOW, and
  hitting the limit *or* a broken or unplugged wire both read HIGH, so a wiring
  fault stops the motor instead of letting it drive past the end stop.
- If the switch wires run alongside the motor cable, twist each signal wire with
  its ground wire.

## Firmware notes

- Configure GPIO32 and GPIO33 (and GPIO25 if used) as **inputs only**. The
  internal pull-up is optional since the board already has 2k pull-ups. Never
  drive these pins as outputs: driving one HIGH while its switch is closed shorts
  the GPIO to GND through the MOSFET's body diode. LilyGO's factory demo drives
  them as PWM outputs at startup, so don't run it with switches connected.
- With NC switches, "at limit" = HIGH.
- Debounce in software (a few ms is enough for a microswitch).
