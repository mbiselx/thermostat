# Notes for agents working on this repo

Start with [`README.md`](README.md) (what this is and how to build it) and
[`docs/design-decisions.md`](docs/design-decisions.md) (why). Don't undo a
decision recorded there without saying so: several obvious-looking
alternatives (limit switches, StallGuard-only stall detection, SpreadCycle,
calibrating on every boot) were tried and dropped for a reason.

## Firmware (`fw/`)

- ESPHome config `fw/thermostat.yaml` plus the external component
  `fw/components/valve_actuator/` (C++ with Python config schemas). The full
  option reference is in `fw/README.md`; keep it in step with `valve.py`,
  `climate.py` and the YAML when options change.
- All motion settings are in valve degrees, after the 59:15 gearbox
  (`gear_ratio: 3.933`). Positive = opening = counter-clockwise.
- Check a change without hardware:
  ```sh
  pip install esphome
  cd fw && cp secrets.yaml.example secrets.yaml   # dummy values are fine; the API key must be 32 bytes base64 (openssl rand -base64 32)
  esphome config thermostat.yaml
  ```
  `esphome compile` needs the ESP32 toolchain download, which may not be
  reachable from a sandbox. Real builds and flashing happen on the owner's
  machine (`esphome run thermostat.yaml --device thermostat.local`).
- Never claim a hardware test happened unless the owner reports one. Keep run
  current at or below 1.2 A: the driver has no heatsink.

## Hardware (`hw/`)

- `hw/cad/freecad/` is the source of truth; STEP and STL are exports. If a part
  changes, re-export both and update the tables in `hw/cad/README.md`.
- `hw/BOM.md` lists everything to buy and print.

## Home Assistant (`sw/`)

Automations are applied in Home Assistant by hand; the YAML here is a
reference copy. Entity IDs are listed in `sw/README.md`.
