# ZwiftControlls

ESP32 tools for connecting to **Zwift Click** (and Zwift Ride) BLE controllers using NimBLE. Read button presses, forward them as a BLE relay, or integrate them into your own projects.

## Sketches

### Zwift_controls

Standalone scanner and logger. Connects to one or both Zwift Click controllers and prints button presses to Serial.

- **Single mode:** Connects to left (8) first; falls back to right (7) if no events within timeout.
- **Dual mode** (`USE_DUAL_CONNECTION 1`): Connects to both controllers. Left = `[L]` arrows, right = `[R]` YZBA+.
- Keep-alive (Info-Request every 5 s, RideOn every 30 s) prevents Clicks from sleeping.
- Compact logging: one line per keypress, e.g. `[L] KEY: LEFT` or `[R] KEY: A, B`.

### ZwiftControlsRelay

Central + Peripheral in one sketch. Connects to both Clicks **and** acts as a BLE relay server so a tablet or phone can use the same buttons through the ESP32.

- **Central:** Same dual-controller logic as Zwift_controls (left first, then right, with timeouts and rescan).
- **Relay server:** BLE service `FC82` with Measurement (Notify), Control (Write), Response (Indicate). Tablets connect to the ESP32 and receive the same notifications as a direct Click connection.
- Neutral manufacturer data (`0xFFFF`) by default; Zwift-style (`0x094A`) available as option.
- Heap-safe: all BLE callbacks are static instances, clients freed via `deleteClient()`, no `new` per connection.
- FreeRTOS mutex protects Serial output from interleaving in concurrent BLE callbacks.

## Protocol

The Zwift Click / Ride BLE protocol is documented in detail:

- [`Zwift_controls/PROTOCOL_REFERENCE.md`](Zwift_controls/PROTOCOL_REFERENCE.md) – Service UUIDs, handshake, keypress format, button bitmasks.
- [`Zwift_controls/PROTOCOL_ZWIFT_PLAY.md`](Zwift_controls/PROTOCOL_ZWIFT_PLAY.md) – Zwift Play / Ride specifics.

### Quick summary

| Property | Value |
|----------|-------|
| BLE Name | `"Zwift Click"` / `"Zwift SF2"` (Ride) |
| Service | `0xFC82` (new) or `00000001-19ca-…` (legacy) |
| Handshake | Write `"RideOn"` → Indicate `"RideOn"` |
| Keep-alive | `0x00 0x08 0x00` every 5 s, `"RideOn"` every 30 s |
| Keypress | Notify byte `0x23` + protobuf varint field 1 = button bitmask |
| Pressed | Bit = 0 (inverted logic) |
| Device IDs | 8 = left (arrows), 7 = right (YZBA+), 9 = Click |

### Button map

| Bit | Button | Bit | Button |
|-----|--------|-----|--------|
| 0x01 | LEFT | 0x10 | A |
| 0x02 | UP | 0x20 | B |
| 0x04 | RIGHT | 0x40 | Y |
| 0x08 | DOWN | 0x100 | Z |
| 0x200 | SHFT_UP_L | 0x2000 | SHFT_UP_R |
| 0x400 | SHFT_DN_L | 0x4000 | SHFT_DN_R |
| 0x800 | POWERUP_L | 0x10000 | POWERUP_R |
| 0x1000 | ONOFF_L | 0x20000 | ONOFF_R |

## Requirements

- **Hardware:** ESP32 (any variant with BLE support)
- **Library:** [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- **IDE:** Arduino IDE or PlatformIO

## Setup

1. Install NimBLE-Arduino via Library Manager or PlatformIO.
2. Open the desired sketch (`Zwift_controls/Zwift_controls.ino` or `ZwiftControlsRelay/ZwiftControlsRelay.ino`).
3. Flash to your ESP32.
4. Open Serial Monitor at 115200 baud.
5. Turn on your Zwift Click(s) – the ESP32 will discover and connect automatically.

## Configuration

All settings are at the top of each `.ino` file:

| Setting | Default | Description |
|---------|---------|-------------|
| `TARGET_NAME` | `"Zwift Click"` | BLE name filter. Use `"Zwift SF2"` for Ride. |
| `SCAN_PHASE_TIMEOUT_MS` | `8000` | How long to wait per scan phase before moving on. |
| `KEEPALIVE_INTERVAL_MS` | `5000` | Info-Request interval. `0` = disabled. |
| `RIDEON_KEEPALIVE_MS` | `30000` | RideOn interval. `0` = disabled. |
| `USE_DUAL_CONNECTION` | `1` | (Zwift_controls only) `1` = both, `0` = single. |
| `RELAY_MFG_DATA` | `0xFFFF` neutral | (Relay only) Change to `0x094A` for Zwift-style. |

## Connection flow

```
Boot
 ├── Start Relay server (FC82)     [Relay only]
 └── Scan for left (8)
      ├── Found → Connect → Handshake → Scan for right (7)
      │                                   ├── Found → Connect → Both running
      │                                   └── Timeout → Only left active
      └── Timeout (8s) → Scan for right (7)
                           ├── Found → Connect → Only right running
                           └── Timeout → Rescan from start
```

On disconnect, all clients are cleaned up and scanning restarts automatically.

## Further documentation

- [`ZwiftControlsRelay/DOCUMENTATION.md`](ZwiftControlsRelay/DOCUMENTATION.md) – Detailed technical docs (state machine, heap safety, serial output, relay behavior).
- [`ZwiftControlsRelay/INTEGRATION_PLAN.md`](ZwiftControlsRelay/INTEGRATION_PLAN.md) – Plan for integrating Zwift Click as a sensor type into BluetoothBikeSensorServer and Homewind.

## License

MIT
