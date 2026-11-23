# Thermostat

This repo contains the files for my DIY Home Assistant-enabled thermostat project.

## Motivation
I want to be able to control the temperature in my studio using home assistant. Sadly, the valves of the radiators in my studio are not standard thermostatic valves, and so I cannot use COTS termostats for this. Because I am in a rental building and cannot locate the water shutoff valve for my apartment, I cannot simply replace the valve with something more modern either. So I'm making my own DIY thermostat. 

## The Plan
The valve on my radiators are rotationally actuated, and require a fairly high torque (since they are pretty old).
To actuate one, I am goign to use a NEMA17 stepper motor, driven by a LillyGO TTGO T-Motor ESP32 driver board.

### Project Layout

```text
thermostat/ 
├── fw/         # on-device firmware
├── hw/         # hardware related files 
|  ├── cad/         # FreeCAD sources, STEP/STL exports, assembly notes
|  ├── datasheets/ 
|  ├── BOM.md
|  ├── 3d-printed-gears.md  # gear reduction options and printing tips
|  └── limit-switches.md  # limit switch wiring on the T-Motor P1 header
└── sw/         # home assistant integration 
```         