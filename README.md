# RBgrid for ESPHome

The RBgrid metering core as an ESPHome component. An ESP32 hub with rbAmp measurement modules becomes an energy monitor
that Home Assistant talks to over the ESPHome native API.

Everything is metered, stored and priced on the hub:
- live power per circuit and 15-minute periods;
- history on the hub's flash;
- your tariff plan, dynamic (spot) prices, cheap windows with a price notification, import peaks;
- remote ESP-NOW nodes.

Home Assistant is the interface.

- **Install:** flash the ready-made image from the browser, then set Wi-Fi over USB (Improv) or the hub's setup access
  point. Steps: [`esphome/README.md` → Install](esphome/README.md#install).
- **Home Assistant:** add the hub in the ESPHome integration, then install **RBgrid** via HACS (custom repository
  <https://github.com/rb-amp/rbgrid-ha>, category Integration).
- **Your own build / reference:** [`esphome/README.md`](esphome/README.md) covers the YAML, entities, updating and
  troubleshooting. The `rbgrid_cmd` reference is [`esphome/RBGRID_CMD.md`](esphome/RBGRID_CMD.md).

**Hardware:** an ESP32 hub with **16 MB flash and PSRAM**. Ready-made images exist for ESP32-WROVER and ESP32-S3 N16R8.
Measurement comes from rbAmp I²C modules (<https://www.rbamp.com/?utm_source=github&utm_medium=referral&utm_campaign=rbgrid>).

**Licence:** MIT (see `LICENSE`). Third-party parts keep their own licences (see `NOTICE`).
