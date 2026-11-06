# ESPAXA
 ESPHome external component for the AXA Remote 2.0 electric window opener.
 This is a rewrite based on galagaking/espaxa.

 Main features:
  - free positioning, estimated from travel time
  - exposes the current operation (opening / closing / idle)
  - detects opening / closing with the buttons on the AXA itself
  - optional lock sensor, locked only when the AXA reports 211 Strong Locked
  - when the AXA does not confirm the lock, stops it and closes once more
  - non-blocking serial communication, ESPHome stays responsive
  - native Home Assistant API, no MQTT or deep sleep needed

## Usage

```yaml
external_components:
  - source: github://ellepdesk/espaxa

uart:
  tx_pin: GPIO21
  rx_pin: GPIO20
  baud_rate: 19200
  stop_bits: 2

cover:
  - platform: espaxa
    name: None
    # Defaults, tune to your window
    # unlock_duration: 10s   # OPEN until the chain starts moving
    # open_duration: 50s     # chain travel, closed to fully open
    # lock_duration: 45s     # fully closed until locked
    # poll_interval: 30s     # STATUS poll while idle
    # moving_poll_interval: 2s
    # release_before_close: true  # OPEN before CLOSE, releases the AXA
    lock:
      name: "Lock"
```

See [espaxa-cover.yaml](espaxa-cover.yaml) for a complete ESP32-C3 configuration
and [docs/](docs/) for the schematic.
