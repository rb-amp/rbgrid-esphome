# Changelog

## v1.2.0 — 2026-11-11

First release of RBgrid for ESPHome.

### What it does
- **The RBgrid metering core as an ESPHome external component:**
  - rbAmp I²C modules, up to 8 on one bus;
  - live power per point, 15-minute periods;
  - history on the hub's flash;
  - the tariff plan with time-of-use zones, taxes, fees, VAT and net metering;
  - remote ESP-NOW nodes.
- **Dynamic (spot) prices:**
  - Energy-Charts, Nord Pool, Elpriset just nu, Hva koster strømmen, EnergyZero, Octopus Agile, or Home Assistant as the source;
  - a primary and a backup source, then the plan;
  - a 40-day log of every applied price.
- **Cheap windows (up to 8 rules) and a price notification.** The hub switches nothing itself; HA automations read the
  windows and flags.
- **Import peaks** per billing cycle, for capacity tariffs (metering only).
- **Home Assistant:**
  - entities for the system and 12 point slots (a slot's entities start disabled until a point takes it);
  - the `rbgrid_cmd` action for all configuration;
  - `esphome.rbgrid_event` events with a `seq` cursor.
- **Ready-made images** for ESP32-WROVER and ESP32-S3 N16R8:
  - install from the browser;
  - Wi-Fi over Improv or the setup access point (`rbgrid-hub-xxxxxx`, password `RBGRID123`; change it after adoption);
  - updates over the air.
- **Planned restarts** (OTA, restart from HA) keep the open hour and the lifetime totals.

### Known behaviour
- While the core is starting (a few seconds after a boot), `rbgrid_cmd` answers `[code=STARTING]`; repeat the command.
