# 3D-printed gear reduction for the valve actuator

Goal: 1:2 minimum, ideally 1:3 to 1:4, silent, durable, compact, next to a hot radiator.
Motor: SY42STH38-1684A (36 N·cm holding at 1.68 A; datasheet in `hw/datasheets/`).
At the ~1.2 A the TMC2209 can sustain without a heatsink, expect roughly **0.2 N·m usable** at low speed.

## TL;DR

- **Recommended: a single-stage helical (or herringbone) spur pair, about 1:4**, motor beside the valve.
  It gives ~0.75 N·m at the valve (about 3.7x what you have now), is the easiest to print well,
  and keeps stall detection and position tracking simple.
- **A worm is only compact and quiet if you accept a high ratio (15:1 to 30:1)**, and then most of the
  motor's work goes into friction. At the 1:3 to 1:4 you asked for, a worm is the *worst* option.
  Its big upside is holding the valve with the motor off.
- **Do not use PLA.** It softens around 55-60 °C, which a radiator valve body can reach.
- Noise will come mostly from the stepper, not the gears. StealthChop plus low speed (the valve has
  no reason to move fast) does more for silence than the gear type does.

## Option comparison

| | Helical/herringbone 1:4 | Worm 20:1 (single start) | Worm 4:1 (4-start) | GT2 belt 1:3 | Printed planetary 1:4 |
|---|---|---|---|---|---|
| Torque at valve (0.2 N·m in) | ~0.75 N·m | 0.8 dry / 1.5 greased | 0.37 dry / 0.55 greased | ~0.57 N·m | ~0.7 N·m |
| Efficiency | 90-95 % | 20 % dry / 38 % greased | 45-67 % | ~95 % | 85-90 % |
| Holds valve unpowered | no (valve friction probably does) | **yes, self-locking** | no | no | no |
| Noise | very low (helical) | low if greased, squeaks dry | low if greased | lowest | medium (many meshes) |
| Wear | low (rolling) | high (pure sliding, heat) | high | low | medium |
| Shape | motor beside valve, ~38 mm centre distance | motor perpendicular, lies along the pipe | same | motor beside valve, needs tensioning | coaxial, adds ~20 mm length |
| Print difficulty | easy | medium (worm needs supports or split) | medium | easy (buy pulleys) | hard (tolerances) |

Numbers: module 1.5, 16 mm worm pitch diameter, PLA-like friction μ≈0.35 dry, ≈0.15 greased.
Worm efficiency = tan λ / tan(λ + φ); self-locking when lead angle λ < friction angle φ.

## The worm, specifically

Your worry about noise is actually the smaller problem. A well-lubricated worm is one of the quietest
gear types because the mesh slides continuously instead of teeth striking each other. The real issues:

1. **Ratio vs efficiency.** A worm's ratio is (wheel teeth / worm starts). To get 1:4 you need a
   multi-start worm with a steep lead, which is no longer self-locking and only reaches ~45-67 %
   efficiency, so it gives *less* torque than a 1:2 spur. A worm only makes sense at 15:1+.
2. **Axial load on the motor.** The datasheet limits axial force on the shaft to **10 N**. At 1.5 N·m
   on a 30 mm wheel, the worm pushes ~100 N along its shaft. The worm must sit on its own shaft with
   bearings (e.g. two 625ZZ, or a thrust washer) and a coupler to the motor, which eats much of the
   compactness advantage.
3. **Wear and heat.** All of the 60-80 % lost power becomes sliding heat at the tooth faces. Printed
   worms wear noticeably without grease, and dry plastic-on-plastic is what squeaks.
4. **Speed.** At 20:1 and a quiet ~300 rpm motor, the valve turns at 15 rpm. That is fine for a
   thermostat, just slow.

If you still go worm: single start, wheel 20-30 teeth, wheel in nylon or PETG and worm in a different
material (or a brass worm), PTFE grease, bearings taking the thrust, and a throated (enveloping) wheel
for more contact area if your generator supports it.

**Upside worth knowing:** self-locking means you can disable the driver between moves. No idle
current, no driver heat, no holding hum. With a spur train you can probably do the same, because an
old stiff valve likely won't turn itself back, but that is worth testing.

## How each option affects boot autocalibration

- **Limit switches on the output side are the robust choice with any gearbox.** Sensing on the valve
  side means backlash and gear ratio don't affect where "0 %" and "100 %" are.
- **StallGuard through a reduction** still works with spur/helical gears: a hard stop at the valve
  stalls the motor directly. But the stall torque is multiplied by the ratio, so at 1:4 the motor
  can press ~0.8 N·m into the valve's end stop before detection. Lower the run current during
  homing so a stall can't strip teeth or damage the valve.
- **With a worm, stall detection gets harder.** Friction is a large, temperature- and grease-dependent
  part of the baseline load, so the StallGuard threshold has a narrower, drifting window. At 20:1 the
  stall can also push ~1.5 N·m+ into the valve stop. Use limit switches if you pick a worm.
- StallGuard4 on the TMC2209 only works in StealthChop and above a minimum speed (TCOOLTHRS), so the
  homing move must not be too slow. With a 20:1 worm, a homing speed fast enough for StallGuard may
  be faster than you'd like for silence.
- **Backlash:** approach every target from the same direction (or add a small overshoot-and-return) so
  the percentage is repeatable. This matters more with printed gears than with metal.
- The firmware's gear ratio should be a float; non-integer ratios like 13:51 are fine.

## Design tips for printed gears

**Geometry**
- Module 1.25-1.5 for a 0.4 mm nozzle. Module 1 works but tooth flanks get coarse and strength is
  marginal. Rough Lewis check at 1:4 (15/60 teeth, 8-10 mm face): pinion bending stress ~11 MPa at
  m1, ~4 MPa at m1.5, against ~30 MPa usable for printed PETG/nylon.
- 20° pressure angle. Keep the pinion at 15+ teeth, or use profile shift if you go smaller, to avoid
  undercut that weakens the root.
- Helical, 15-25° helix angle, for quiet running (more teeth in contact at once). Herringbone cancels
  the axial thrust a single helix creates and prints flat without supports.
- Use a "hunting tooth" pair where possible (e.g. 15:59 instead of 15:60) so each tooth meets every
  tooth of the other gear and wear evens out.
- Add backlash on purpose: 0.1-0.2 mm extra centre distance, or thin the teeth in the generator.
  Printed gears at exact theoretical spacing bind and squeal.
- Face width 8-12 mm is plenty here.

**Printing**
- Print gears flat (axis vertical). Layer lines then run across the teeth in-plane, the strongest
  orientation for bending.
- 0.12-0.16 mm layers, slow outer walls, 4-6 perimeters (or 100 % infill) so teeth are solid plastic.
- Put the Z seam where it doesn't land on a tooth flank (aligned to the back of a hub, or at a root).
- Chamfer the bottom edge (0.4-0.5 mm) to remove elephant's foot, which otherwise rubs on the mesh.
- Print one test pair first and check mesh by hand before printing the housing.

**Materials** (radiator heat rules out PLA)
- **Nylon (PA, PA-CF, PA12)**: best for gears. Tough, self-lubricating, quietest, good to ~80-100 °C.
  Needs dry filament and a hotter printer.
- **PETG**: acceptable and easy. Softens around 75-80 °C, so keep it off the valve body itself.
- **ASA/ABS**: good heat resistance (~95-100 °C), decent wear.
- Pair different materials (e.g. nylon pinion, PETG or ASA wheel). Dissimilar plastics wear and squeak less.

**Mechanics**
- **Buy a metal pinion, print the big gear.** Brass/steel spur gears with a 5 mm bore and set screw
  are cheap in module 1 / 1.25 / 1.5. The pinion sees 3-4x more tooth contacts than the wheel and a
  printed D-bore on a motor shaft tends to slip or crack.
- If printing the pinion anyway: captive M3 nut + set screw onto the shaft flat, not just a D-hole.
- Keep the pinion close to the motor flange. The datasheet limit is 28 N radial at 20 mm from the
  flange, and at 1.2 A the mesh pushes ~30 N on a 16 mm pinion.
- Support the output gear on its own bearing or bushing so the valve stem only sees torque, not side load.
- Lubricate with PTFE or silicone grease (plastic-safe, e.g. Super Lube). Avoid petroleum greases on ABS/ASA.

**Tools that generate these**
- FreeCAD Gears workbench (spur, helical, herringbone, worm, with profile shift).
- OpenSCAD: BOSL2 `gears.scad` or the "Getriebe/gears" library (worm and herringbone included).
- Fusion 360 / Onshape: built-in spur gear script, or the Onshape gear FeatureScript for helical.

## Fallbacks if space or noise is still a problem

- **GT2 belt, 20T metal pulley to 60T printed pulley (1:3):** quietest option, tolerant of misalignment,
  cheap off-the-shelf parts. Needs a tensioning slot and a bit more room.
- **NEMA17 with an integrated planetary (e.g. 5.18:1):** off the shelf, coaxial, very compact,
  rated for several N·m. Not printed, but removes all the gear design risk.

Inferred, not measured: the valve's actual breakaway torque. A quick measurement (torque wrench or a
lever arm and luggage scale on the valve handle) would tell you whether 1:3 is enough or whether to
go straight to 1:4 or higher.
