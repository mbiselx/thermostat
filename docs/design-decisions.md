# Design decisions

What was decided, when, and why, in roughly the order it happened. The
current state of each part is described in its own README; this file is for
the reasons, including the things that were tried and dropped.

## The problem (2026-10)

- **Goal.** Control the studio's heating from Home Assistant: turn it off while
  a window is open, keep it warm when someone's home, temperate at night, cool
  when nobody's there, and still allow setting a temperature by hand.
- **Why build an actuator.** The radiators have old rotary valves, not
  standard thermostatic valve bodies, so no off-the-shelf smart thermostat head
  fits. Replacing the valve isn't an option: it's a rental, and the flat's
  water shutoff can't be found.
- **The valve.** An 11 mm round spindle that turns about 373° (measured
  373.1° on 2026-11-19) from closed to open, stiff because it's old. A
  Ø19.5 mm bonnet with two flats and a lug, then the valve body and a union nut,
  none of which turn. Space around it is tight: a pipe runs sideways to the
  radiator, and the wall is close.

## Motor and board (2026-10-17)

- **NEMA17 stepper (SY42STH38-1684A) on a LilyGO TTGO T-Motor.** The board is
  made to sit on the back of a NEMA17 and has everything needed: an ESP32 for
  Wi-Fi and Home Assistant, a TMC2209 driver that's quiet and configurable over
  UART, an MT6816 magnetic encoder that reads the motor shaft, and three
  buttons. A stepper gives position without extra sensors, and holds no current
  when idle if the valve's friction keeps it in place.
- **The driver's current limit.** The board's 120 mOhm sense resistors allow
  about 1.64 A RMS, but without a heatsink the TMC2209 overheats within seconds
  above about 1.2 A (the torque-test build hit the driver's 120 °C warning after
  ~15 s at 1.6 A). So the firmware runs at 1.0 A and never offers more than
  1.2 A on its sliders.
- **Board quirks found on the schematic** (LilyGO's examples get some of these
  wrong): the sense resistors are 120 mOhm, not 110; the SPREAD pin inverts the
  chopper-mode bit, so firmware holds it low and sets the mode in software;
  DIAG only lights an LED and doesn't reach the ESP32, so StallGuard must be
  polled over UART; P1's EN/STP/DIR pins aren't the driver's control lines but
  spare inputs; the TMC2209 only answers on UART once 9 to 12 V is on P1.

## Gear reduction (2026-10-17)

- **Direct drive was too weak.** A torque test on 2026-10-17 (PlatformIO
  firmware, `archive/torque-test` branch, up to 1.6 A) showed the motor was too
  weak to turn the valve with the motor directly on the spindle. The motor gives roughly 0.2 N·m at
  the current the driver can sustain, so a reduction of about 1:3 to 1:4 was
  needed.
- **Single-stage helical spur pair, not a worm.** Research in
  [`hw/3d-printed-gears.md`](../hw/3d-printed-gears.md). A worm is only worth
  it at 15:1 or more, where it's self-locking but 20 to 40 % efficient; at 1:4
  it needs a multi-start worm and gives less torque than a 1:2 spur. A worm on
  the motor shaft would also need its own bearings, because the motor allows
  only 10 N axial load. Belts and printed planetaries were the fallbacks. A
  spur pair also keeps stall detection simple, because a hard stop at the valve
  stalls the motor directly.
- **Herringbone, 15:59 teeth, normal module 1.25, 20° pressure angle, 22.33°
  helix, 50 mm centres, 12 mm face.** Helical teeth run quieter (more teeth in
  contact); herringbone cancels the axial thrust a single helix puts on the
  motor shaft, and prints flat without supports. 15:59 is a hunting-tooth pair
  (wear evens out) giving 3.933:1, and the helix angle was chosen to make the
  centre distance exactly 50 mm. Module 1.25 suits a 0.4 mm nozzle.
- **Both gears printed, in PETG.** The research suggested a bought metal
  pinion; the printed one works for now. PLA was ruled out because it softens
  around 55 to 60 °C, close to what the valve body reaches.
- **0.07 mm layers for the gears** (not the 0.12 to 0.16 mm the research
  suggested): Michael's working prototype (built by 2026-11-14) was printed at
  0.07 mm and runs very quietly and smoothly.
- **The big gear sticks out past the housing on purpose**, so the valve can
  still be turned by hand during a power cut or a Wi-Fi problem.

## Firmware platform: ESPHome with a custom component (2026-10-17)

- **Requirements:** calibrate itself at boot, buttons on the board for 0, 50
  and 100 %, and Home Assistant seeing a valve with a percentage.
- **Options considered:** plain PlatformIO firmware talking MQTT, a custom
  Home Assistant integration, or ESPHome. **Chosen: ESPHome with an external
  C++ component** (`fw/components/valve_actuator`). ESPHome gives Wi-Fi, OTA,
  the native Home Assistant API, entities and the board buttons for free in
  YAML, while the motor logic that ESPHome doesn't have (TMC2209 setup over
  UART, a step ramp, stall detection, calibration) lives in C++. It was the
  most flexibility for the least plumbing.
- **Own TMC2209 driver code** rather than an existing ESPHome TMC2209
  component: those rely on the DIAG pin for stall events, which this board
  doesn't wire to the ESP32.
- **`api: reboot_timeout: 0s`.** ESPHome's default reboots the device after
  15 minutes without Home Assistant, and every reboot could mean another drive
  into the end stop. The valve should keep working through a Home Assistant
  outage.
- The PlatformIO bring-up firmware (serial console, torque test) stays on the
  `archive/torque-test` branch.

## Finding the end stops

This went through three designs.

1. **Limit switches (planned 2026-10-17, dropped 2026-11-14).** The original
   plan, and what the gear research recommended: two microswitches on the
   output side, wired to P1's spare inputs
   ([`hw/limit-switches.md`](../hw/limit-switches.md)). They were dropped for
   the second prototype: integrating them into the housing proved too time
   consuming, and stall detection solved the same problem with no extra parts
   or wiring. The firmware still
   accepts them (`open_limit`, `closed_limit`).
2. **StallGuard (2026-11-14).** Drive towards closed until the TMC2209's load
   reading drops below a share of its free-running value. This failed on the
   real build: the big gear holds the round spindle with a grub screw only, so
   at the end stop the gear slips on the spindle instead of the motor stalling.
   While it slips, the load the motor sees doesn't change: StallGuard stayed at
   85 to 110 throughout (measured 2026-11-19). Grinding a flat onto the spindle
   would fix the slip, but Michael didn't want to grind the valve yet.
3. **Encoder-detected stall at a low current (2026-11-19, current).** The
   calibration runs the motor at a reduced current (0.6 A) so that at the end
   stop the motor stalls before the gear's grip gives way. The MT6816 sees the
   motor fall behind its steps (3 full steps = stall). 0.6 A was found to stall
   cleanly without slipping. StallGuard is still checked as a second signal.
   The current is a slider in Home Assistant because the right value depends on
   how tight the grub screw is.

Related choices:

- **Manual "Set closed here" / "Set open here" buttons**, so the end stops can
  be set by turning the valve by hand without ever driving into them. They
  were added at the same time as the encoder because the stall search was
  unreliable.
- **Configured travel 360°, measured 373°.** 100 % stops a little short of the
  real open end so normal moves never press the gear into the stop.
- **Search limit 380°** (`homing: max_travel`). Just more than the full travel,
  so a failed search can't wind far past the end. It was 480° on 2026-11-14,
  tightened to 380° once the travel was measured.
- **Calibration only finds the closed end** and keeps the saved travel; it
  only searches for the open end if the travel isn't known.
- **No calibration on every boot.** The position is saved with the encoder
  angle after each move. At boot it's trusted if the encoder agrees, and the
  valve only searches for the closed end when it can't be trusted. Every drive
  into the stop risks slipping the gear, so it should be rare.

## Quiet operation (2026-11-19)

- **Full StealthChop** (`spread_cycle: false`). The radiator is in the
  bedroom. SpreadCycle gives more torque but is audible.
- **25 °/s** at the valve for moves and calibration. Faster was audible; the
  valve has no reason to move fast.
- **Motor unpowered between moves** (`hold_when_idle: false`). The valve's
  friction holds it, the driver stays cool, there's no holding hum, and the
  valve can be turned by hand. The encoder follows hand turns and the position
  is saved once the valve has been still for 2 s.

## Power cuts (2026-11-19)

- If the encoder doesn't agree with the saved position at boot (the valve was
  moving when power went, or it was turned while off), the valve **drives
  closed to find zero, then opens to 5 %**, and raises **Valve needs
  attention** in Home Assistant.
- 5 % rather than staying closed so the radiator can't freeze while nobody
  notices; an alert because someone should check the valve and put it where
  they want it. **Dismiss alert** or a calibration clears it.

## Thermostat in the firmware (2026-11-19)

- **Why not a Home Assistant `generic_thermostat`.** Home Assistant's climate
  dashboard only lists climate entities, and the valve was a valve entity. A
  climate entity in the firmware puts the thermostat on the device itself,
  remembers its state through power cuts, and keeps holding the valve where
  it was if Home Assistant goes away.
- **PI controller built to move the valve as little as possible.** Every move
  makes noise and wears the gears. It runs once a minute, ignores errors under
  0.2 °C for the integral, and only moves the valve in steps of at least 5 %.
  The integral is saved so a reboot doesn't restart the learning.
- **Presets Home 23.5, Sleep 20.5, Away 18.5 °C**, from Michael's targets
  (23 to 24 °C when home, 20 to 21 °C asleep, 18 to 19 °C away). Michael found
  that a fixed 25 to 30 % holds the night temperature with few moves, and 5 %
  holds the away temperature, so Sleep and Away start the valve there.
- **Direct valve commands hold the valve.** If someone moves the valve by
  hand, with the buttons or from Home Assistant, the thermostat stops fighting
  it until its mode, target or preset changes. This is why the window
  automation turns the thermostat off and on instead of setting the valve.

## Thermostat as the main entity, sensors chosen in HA (2026-11-23)

- **`name: None`**, so the climate entity is the device's main entity and
  takes the device's name. A second radiator would be the same firmware with a
  different `friendly_name`.
- **Temperature sensors are picked in Home Assistant**, in text entities on the
  device, instead of being compiled in. Each preset can follow its own sensor:
  Home the SwitchBot Hub 3 in the room, Sleep a meter by the bed, Away a meter
  by the window (the coldest spot, so the room never drops too far). Changing
  one takes effect at once without reflashing.
- **No sensor on the board.** The thermostat relies on Home Assistant for the
  temperature; if it's unavailable, the valve stays where it is. A local
  sensor is on the to-do list.

## Home Assistant automations (2026-11-19)

- **Window pause:** after any window has been open for 15 s, turn the
  thermostat off (which closes the valve); when all windows have been closed
  for 30 s, turn it back on, unless someone changed it in the meantime. The
  15 s and 30 s stop a quick airing or a sensor bounce from toggling the
  heating.
- The first version set the valve to 0 % and back. Once the thermostat moved
  into the firmware, that would have held the valve and locked the thermostat
  out, so it was rewritten to use climate off/on. Off is also remembered by the
  firmware, so a power cut during the pause doesn't reopen the valve.
- **Automations are applied in Home Assistant by hand.** Michael applies Home
  Assistant config in Home Assistant directly; the Home Assistant connector
  used during development can read states and call actions but can't create
  automations. The YAML in [`sw/automations/`](../sw/automations/) is a
  reference copy to paste.
- **Schedule and presence: proposed, not built.** The proposal: decide
  "anyone home" rather than tracking who (the door sensor plus motion sensors,
  "wasp in a box": after the door closes, any motion means someone's inside),
  switch Away / Sleep / Home on those events and on the phone alarm, and let a
  manual change hold until the next switch. No calendar, because the schedule
  varies week to week and keeping a calendar up to date was a burden Michael
  wanted to avoid.
