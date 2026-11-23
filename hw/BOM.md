# Bill of materials

Everything for one valve actuator. Sizes, screw counts and print settings come
from [`cad/README.md`](cad/README.md), which has the detail.

## Electronics

| Item | Qty | Notes |
| --- | --- | --- |
| LilyGO TTGO T-Motor | 1 | ESP32 + TMC2209 + MT6816 encoder, mounts on the back of a NEMA17. This build is the **MOSFET** variant of the P1 inputs (three BSS123W next to P1); only matters if you add limit switches. [Board repo and schematic](https://github.com/Xinyuan-LilyGO/T-Motor) |
| Magnet for the encoder | 1 | Diametrically magnetised. It's simply laid on the back end of the motor shaft and held there by its own pull, under the MT6816; no glue. Without it the firmware still runs on step counting, but can't follow hand turns, see the motor stall during calibration, or check its position after a power cut |
| NEMA17 stepper, SY42STH38-1684A | 1 | 1.8°, 1.68 A, 36 N·cm. [Datasheet](datasheets/SY42STH38-1684A.pdf). Run at 1.0 A (0.6 A while calibrating) |
| 12 V DC power supply | 1 | 9 to 12 V into P1 pins 1 (V+) and 2 (GND) |
| Cable or connector for P1 | 1 | JST 6-pin, 2 wires used |
| USB-to-serial downloader for ESP32 | 1 | First flash only. One that drives EN and G0 (e.g. M5Stack's ESP32 downloader) puts the board into the bootloader by itself |

## Mechanical

| Item | Qty | Notes |
| --- | --- | --- |
| M3 screws, motor to housing | 4 | About M3 × 6 to 8 (2 mm motor plate) |
| M3 × 20 socket head screws | 3 | Join the two housing halves |
| M3 square nuts, DIN 562 (5.5 × 1.8 mm) | 3 | In the back housing half |
| Grub screw | 1 | Holds the valve gear on the spindle |
| Acrylic sheet, about 118 × 90 × 5 mm | 1 | Front window. Measure your printed housing first |
| PTFE or silicone grease | a little | Plastic-safe, for the gear teeth |

## Printed

PETG (ASA or nylon also fine). **Not PLA**: it softens near the valve's
temperature.

| Part | File | Qty | Settings |
| --- | --- | --- | --- |
| Valve gear, 59 teeth | [`cad/stl/valve_gear.stl`](cad/stl/valve_gear.stl) | 1 | 0.07 mm layers, solid |
| Pinion, 15 teeth | [`cad/stl/pinion.stl`](cad/stl/pinion.stl) | 1 | 0.07 mm layers, solid |
| Housing, front | [`cad/stl/housing_front.stl`](cad/stl/housing_front.stl) | 1 | 0.2 mm, 20 to 30 % infill, supports |
| Housing, back | [`cad/stl/housing_back.stl`](cad/stl/housing_back.stl) | 1 | 0.2 mm, 20 to 30 % infill |
| Fit tests | [`cad/stl/fit-tests/`](cad/stl/fit-tests/) | 1 each | Print these first |

## Not used any more

| Item | Notes |
| --- | --- |
| Microswitch (TRU Components TC-9122112) | Limit switches were dropped for the second prototype. [Datasheet](datasheets/datenblatt-2280528-tru-components-tc-9122112-mikroschalter-250-vac-5-a-1-x-einein-tastend-1-st.pdf), wiring in [`limit-switches.md`](limit-switches.md) |
