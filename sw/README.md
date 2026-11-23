# Home Assistant

The firmware talks to Home Assistant through ESPHome's native API, so there's
no custom integration to install. Adding the device is in the
[top-level README](../README.md#7-add-it-to-home-assistant), and its entities
are listed in [`fw/README.md`](../fw/README.md#home-assistant).

This folder holds the Home Assistant side that lives in Home Assistant itself:
automations. They're kept here as reference copies, applied by pasting them
into Home Assistant (Settings > Automations & scenes > Create automation >
⋮ > Edit in YAML), not loaded from this repo.

## Entities this setup uses

The device is named "Room thermostat" in Home Assistant, so:

| Entity | What |
| --- | --- |
| `climate.room_thermostat` | The thermostat (the device's main entity) |
| `valve.room_thermostat_radiator_valve` | The valve, 0 % closed to 100 % open |
| `binary_sensor.room_large_window`, `binary_sensor.room_small_window`, `binary_sensor.kitchen_window` | Window contact sensors (SwitchBot, over Bluetooth) |
| `sensor.room_hub3_a50e_temperature`, `sensor.room_hub3_a50e_humidity` | SwitchBot Hub 3: the main temperature sensor and the humidity shown on the thermostat |
| `sensor.room_meter_bed_temperature` | Meter by the bed, used in Sleep |
| `sensor.room_meter_tv_temperature` | Meter by the window, used in Away |

With a different device name or other sensors, change the IDs in the
automations, and the thermostat's sensor fields on the device page (or the
`substitutions:` in `fw/thermostat.yaml`).

## Automations

### [`heating_window_pause.yaml`](automations/heating_window_pause.yaml)

Pauses the heating while a window is open:

- After any window has been open for 15 s, it turns the thermostat off, which
  closes the valve. It does nothing if the thermostat is already off.
- When every window has been closed for 30 s, it turns the thermostat back on
  in the mode it had, unless someone changed it in the meantime.

It turns the thermostat off rather than setting the valve to 0 %, because a
direct valve command holds the valve and keeps the thermostat out until its
next change. Off is remembered by the firmware, so a power cut during the
pause doesn't reopen the valve. If Home Assistant restarts while a window is
open, the automation's wait is lost and the thermostat stays off until it's
turned on again.

The kitchen window counts as part of the studio; drop it from both lists if
it shouldn't pause the heating.

## Not built yet

Schedules and presence (Away while nobody's home, Sleep at night, Home
otherwise, with a manual change holding until the next switch). The proposal
and its reasons are in
[`docs/design-decisions.md`](../docs/design-decisions.md#home-assistant-automations-2026-11-19).
