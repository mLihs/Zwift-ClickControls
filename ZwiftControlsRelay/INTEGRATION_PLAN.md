# Zwift Click Integration Plan

Integration of the Zwift Click BLE controller as a new sensor type into:
- **Step 1:** BluetoothBikeSensorServer (BBS) library
- **Step 2:** Homewind application (discovery → NVS → reconnect → Web UI)

---

## Background: Zwift Click Protocol

| Property | Value |
|----------|-------|
| BLE Name | `"Zwift Click"` |
| Service UUID | `0xFC82` (or legacy `00000001-19ca-4651-86e5-fa29dcdd09d1`) |
| Measurement Char | `00000002-…` (Notify) |
| Control Char | `00000003-…` (Write) |
| Response Char | `00000004-…` (Indicate) |
| Manufacturer ID | `0x094A` (Zwift, Inc.) |
| Device ID | Byte 2 of MfgData: `8` = left (arrows), `7` = right (YZBA) |
| Handshake | Write `"RideOn"` to Control → Indicate reply; then subscribe Measurement |
| Keep-alive | Write `0x00 0x08 0x00` every 5 s; `"RideOn"` every 30 s |
| Button data | MsgId `0x23`, protobuf field 1 varint = button bitmask (pressed = bit low) |

**Key difference vs standard BLE sensors (HR, CSC, Power):**
The Zwift Click requires a **handshake** (RideOn + Info-Request) and **keep-alive** writes on the Control characteristic. Standard sensors only need `subscribe(true)` on one characteristic. The BBS connection flow must be extended for this.

---

## Step 1: BluetoothBikeSensorServer (BBS) Integration

### 1.1 Files to modify

| File | Change |
|------|--------|
| `BBSConfig.h` | Add `BBS_COMPILE_ZWIFT_CLICK` flag |
| `BluetoothBikeSensorServer.h` | Add `SENSOR_TYPE_ZWIFT_CLICK` enum, public API functions |
| `SensorRegistry.h` | Extend `SensorTypeConfig` for custom handshake |
| `BluetoothBikeSensorServer.cpp` | Add registry entry, connection logic, mirror func |
| `SensorDataParser.h/cpp` | Add `parseAndLogZwiftClick()`, data structs |
| `BLEScanManager.cpp` (optional) | Scan for FC82 service UUID |

### 1.2 BBSConfig.h – compile flag

```cpp
// Zwift Click (steering + buttons via FC82)
#ifndef BBS_COMPILE_ZWIFT_CLICK
  #define BBS_COMPILE_ZWIFT_CLICK 0   // off by default (not needed for most builds)
#endif
```

### 1.3 BluetoothBikeSensorServer.h – enum and API

```cpp
enum SensorType {
  SENSOR_TYPE_HEART_RATE    = 0x01,
  SENSOR_TYPE_SPEED_CADENCE = 0x02,
  SENSOR_TYPE_POWER_METER   = 0x04,
  SENSOR_TYPE_ZWIFT_CLICK   = 0x08,  // NEW
};
```

Public API additions:

```cpp
#if BBS_COMPILE_ZWIFT_CLICK
// Callback fired on every button-state change (buttonMap bitmask, pressed = bit 0)
typedef void (*ZwiftClickButtonCallback)(uint32_t buttonMap, uint8_t deviceId);
void BluetoothBikeSensorServerSetZwiftClickCallback(ZwiftClickButtonCallback cb);

// Getters
uint32_t BluetoothBikeSensorServerGetZwiftClickButtonMap();
bool     BluetoothBikeSensorServerHasZwiftClick();
#endif
```

### 1.4 SensorRegistry.h – extend for custom handshake

The current `SensorTypeConfig` has `serviceUUID16` and `charUUID16` for simple subscribe-only sensors. Zwift Click needs **three** characteristics + a handshake sequence. Two approaches:

**Option A (recommended): Add optional handshake function pointer**

```cpp
struct SensorTypeConfig {
  SensorType type;
  uint8_t    typeBit;
  const char* shortName;
  const char* serviceName;
  uint16_t   serviceUUID16;     // 0 if using 128-bit UUID
  const char* serviceUUID128;   // NEW: null if using 16-bit UUID
  uint16_t   charUUID16;        // primary notify characteristic
  const char* charUUID128;      // NEW: null if using 16-bit UUID
  void (*parseFunc)(const uint8_t*, size_t);
  void (*mirrorFunc)(const uint8_t*, size_t);
  // NEW: optional custom handshake (called after connect, before subscribe).
  // Returns true on success. If null, default subscribe-only flow is used.
  bool (*handshakeFunc)(NimBLEClient* client, NimBLERemoteService* svc);
  // NEW: optional periodic keep-alive (called from loop). null = none.
  void (*keepAliveFunc)(NimBLEClient* client);
  uint32_t   keepAliveIntervalMs;  // 0 = no keep-alive
};
```

**Option B (simpler, but less generic): Hard-code Zwift Click flow**

Add `if (type == SENSOR_TYPE_ZWIFT_CLICK)` branches in `connectAndSubscribeSensor()`. Less clean but fewer changes to the shared struct.

**Recommendation:** Option A if you plan more custom sensors in the future; Option B for a quick integration.

### 1.5 SensorDataParser – parse function and data

```cpp
// SensorDataParser.h
#if BBS_COMPILE_ZWIFT_CLICK
struct ZwiftClickData {
  uint32_t buttonMap;        // current button bitmask (pressed = bit 0)
  uint8_t  deviceId;         // 7 = right, 8 = left
  bool     valid;
  uint32_t lastTimestamp;
};

void parseAndLogZwiftClick(const uint8_t* data, size_t len);
#endif
```

```cpp
// SensorDataParser.cpp
#if BBS_COMPILE_ZWIFT_CLICK
void parseAndLogZwiftClick(const uint8_t* data, size_t len) {
  if (len == 0 || data[0] != 0x23) return;  // only keypress messages

  // Parse protobuf varint field 1 (same as ZwiftControlsRelay)
  uint32_t buttonMap = 0xFFFFFFFF;
  size_t i = 1;
  bool ok = false;
  while (i < len) {
    uint32_t tag;
    if (!readVarint(data, len, i, tag)) break;
    if ((tag >> 3) == 1 && (tag & 7) == 0) {
      ok = readVarint(data, len, i, buttonMap);
      break;
    }
    // skip other fields...
  }
  if (!ok && len >= 5)
    buttonMap = (uint32_t)data[1] | ((uint32_t)data[2]<<8)
              | ((uint32_t)data[3]<<16) | ((uint32_t)data[4]<<24);

  // Only fire callback if a button is actually pressed (any bit low)
  if (buttonMap == 0xFFFFFFFF) return;

  // Store and callback
  if (context && context->zwiftClickData) {
    context->zwiftClickData->buttonMap = buttonMap;
    context->zwiftClickData->valid = true;
    context->zwiftClickData->lastTimestamp = millis();
  }
  if (context && context->zwiftClickCallback) {
    context->zwiftClickCallback(buttonMap, context->zwiftClickData->deviceId);
  }
}
#endif
```

### 1.6 BluetoothBikeSensorServer.cpp – registry entry

```cpp
#if BBS_COMPILE_ZWIFT_CLICK
  { SENSOR_TYPE_ZWIFT_CLICK, SENSOR_TYPE_ZWIFT_CLICK,
    "ZClick", "FC82 (Zwift Click)",
    0xFC82,                       // serviceUUID16
    nullptr,                      // serviceUUID128 (not needed, 16-bit works)
    0x0000,                       // charUUID16 = 0 → use 128-bit
    "00000002-19ca-4651-86e5-fa29dcdd09d1",  // charUUID128 (Measurement)
    parseAndLogZwiftClick,
    mirrorZwiftClickPacket,       // or nullptr if no relay needed
    zwiftClickHandshake,          // custom handshake
    zwiftClickKeepAlive,          // periodic keep-alive
    5000 },                       // keep-alive interval (ms)
#endif
```

### 1.7 Custom handshake and keep-alive functions

```cpp
#if BBS_COMPILE_ZWIFT_CLICK
// UUIDs for Zwift Click characteristics
static const NimBLEUUID ZCLICK_CTRL_UUID("00000003-19ca-4651-86e5-fa29dcdd09d1");
static const NimBLEUUID ZCLICK_RESP_UUID("00000004-19ca-4651-86e5-fa29dcdd09d1");

// Keep reference for keep-alive writes
static NimBLERemoteCharacteristic* g_zclickCtrl = nullptr;

bool zwiftClickHandshake(NimBLEClient* client, NimBLERemoteService* svc) {
  auto* ctrl = svc->getCharacteristic(ZCLICK_CTRL_UUID);
  auto* resp = svc->getCharacteristic(ZCLICK_RESP_UUID);
  if (!ctrl || !resp) return false;

  // Subscribe to Response (indicate) for RideOn reply
  if (resp->canIndicate()) resp->subscribe(true);

  // Write "RideOn" handshake
  if (!ctrl->writeValue((uint8_t*)"RideOn", 6, true)) return false;
  delay(400);

  // Write Info-Request
  static const uint8_t infoReq[] = { 0x00, 0x08, 0x00 };
  ctrl->writeValue(infoReq, sizeof(infoReq), true);
  delay(200);

  g_zclickCtrl = ctrl;
  return true;
  // Note: the main BBS flow subscribes to the Measurement char AFTER this returns.
}

void zwiftClickKeepAlive(NimBLEClient* client) {
  if (!g_zclickCtrl || !client->isConnected()) return;
  static uint32_t lastKA = 0, lastRO = 0;
  uint32_t now = millis();
  if (now - lastKA >= 5000) {
    lastKA = now;
    static const uint8_t req[] = { 0x00, 0x08, 0x00 };
    g_zclickCtrl->writeValue(req, sizeof(req), true);
  }
  if (now - lastRO >= 30000) {
    lastRO = now;
    g_zclickCtrl->writeValue((uint8_t*)"RideOn", 6, true);
  }
}
#endif
```

### 1.8 Connection flow changes in connectAndSubscribeSensor()

In `BluetoothBikeSensorServer.cpp`, the existing `connectAndSubscribeSensor()` does:
1. Connect → 2. Get service → 3. Get char → 4. Subscribe

For Zwift Click, insert custom handshake between steps 2 and 4:

```cpp
// After getting the service:
const SensorTypeConfig* cfg = getSensorConfig(sensor.type);

if (cfg->handshakeFunc) {
  // Custom handshake (e.g. Zwift Click: RideOn + Info-Request)
  if (!cfg->handshakeFunc(sensor.client, svc)) {
    log("Handshake failed for %s", cfg->shortName);
    sensor.client->disconnect();
    return;
  }
}

// Then subscribe to measurement char (existing flow)
auto* chr = svc->getCharacteristic(charUUID);
chr->subscribe(true, notifyCallback, true);
```

In the `loop()` function (or timer), call keep-alive for connected sensors:

```cpp
// In BBS loop/tick:
for (auto& sensor : connectedSensors) {
  const SensorTypeConfig* cfg = getSensorConfig(sensor.type);
  if (cfg->keepAliveFunc && sensor.client && sensor.client->isConnected()) {
    cfg->keepAliveFunc(sensor.client);
  }
}
```

### 1.9 Scan callback changes

In `MyAdvertisedDeviceCallbacks::onResult()`, add FC82 check:

```cpp
#if BBS_COMPILE_ZWIFT_CLICK
if (advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xFC82))) {
  matchedType = SENSOR_TYPE_ZWIFT_CLICK;
}
// Also match by name as fallback (not all Clicks advertise FC82 in every packet)
if (matchedType == 0 && name == "Zwift Click") {
  matchedType = SENSOR_TYPE_ZWIFT_CLICK;
}
#endif
```

Same for `DiscoveryScanCallbacks::onScanEnd()`.

### 1.10 Summary of BBS changes

| Component | What to add |
|-----------|------------|
| `BBSConfig.h` | `BBS_COMPILE_ZWIFT_CLICK` flag |
| `SensorType` enum | `SENSOR_TYPE_ZWIFT_CLICK = 0x08` |
| `SensorTypeConfig` | `handshakeFunc`, `keepAliveFunc`, `keepAliveIntervalMs`, `serviceUUID128`, `charUUID128` |
| `SENSOR_REGISTRY[]` | New entry for Zwift Click |
| `SensorDataParser` | `parseAndLogZwiftClick()`, `ZwiftClickData`, `readVarint()` |
| `connectAndSubscribeSensor()` | Call `handshakeFunc` if present |
| `loop()` | Call `keepAliveFunc` for connected sensors |
| Scan callbacks | Match FC82 or name "Zwift Click" |
| Public API | `SetZwiftClickCallback()`, `GetZwiftClickButtonMap()`, `HasZwiftClick()` |

---

## Step 2: Homewind Integration

### 2.1 Files to modify

| File | Change |
|------|--------|
| `src/core/BLERelayManager.h` | Add Zwift Click type constant, increase MAX_SENSORS |
| `src/core/BLERelayManager.cpp` | Type mappings, NVS struct, discovery, callbacks |
| `src/web/ApiActions.cpp` | Validate "ZCLICK" type |
| `src/web/WebSocketTelemetry.cpp` | Sensor snapshot includes Zwift Click |
| `webui_src/app.js` | Add sensor type to UI |
| `webui_src/app.css` | Add icon styles |
| `webui_src/index.html` | Add SVG icon variable |

### 2.2 BLERelayManager – type mappings

**`stringToSensorType()`** (BLERelayManager.cpp ~line 727):

```cpp
if (strcmp(typeStr, "HR") == 0 || strcmp(typeStr, "heart_rate") == 0)
  return SENSOR_TYPE_HEART_RATE;
if (strcmp(typeStr, "CSC") == 0 || strcmp(typeStr, "speed_cadence") == 0)
  return SENSOR_TYPE_SPEED_CADENCE;
// NEW:
if (strcmp(typeStr, "ZCLICK") == 0 || strcmp(typeStr, "zwift_click") == 0)
  return SENSOR_TYPE_ZWIFT_CLICK;
```

**`sensorTypeToString()`** (BLERelayManager.cpp ~line 763):

```cpp
case SENSOR_TYPE_ZWIFT_CLICK: return "ZCLICK";
```

### 2.3 NVS storage – extend for 3rd sensor

Current `NVSSensorData` has `sensors[2]` (HR + CSC). To add Zwift Click:

```cpp
#define MAX_SENSORS 3  // was 2

struct NVSSensorData {
  uint8_t version;           // bump to 3
  uint8_t sensorCount;
  uint8_t reserved[2];
  NVSSensorEntry sensors[3]; // was [2]
  uint16_t crc16;
};

// Slot mapping:
//   0 = HR
//   1 = CSC
//   2 = ZCLICK (NEW)
```

**Migration:** In `loadConfigFromNVS()`, if `version == 2`, the loaded data has only 2 sensor entries. Zero-initialize the 3rd entry and save back with version 3.

### 2.4 Discovery flow

No changes needed in the discovery flow itself – BBS handles scanning. But the discovery start needs to include the new type:

**`startDiscovery()`** (BLERelayManager.cpp ~line 497):

```cpp
// When user selects "Zwift Click" type in UI:
case SENSOR_TYPE_ZWIFT_CLICK:
  BikeSensorServerStartDiscovery(SENSOR_TYPE_ZWIFT_CLICK, duration);
  break;
```

**Completion callback:** Already generic – `onDiscoveryComplete()` processes results regardless of type.

### 2.5 Reconnection by MAC

The existing `reconcileSensorTargets()` already handles any sensor type generically via the `_desiredTargets[]` / `_appliedState[]` arrays. As long as `MAX_SENSORS` is increased and the Zwift Click is stored at index 2, the existing pattern works:

```cpp
// In reconcileSensorTargets(), the loop already iterates all sensor types:
for (int i = 0; i < MAX_SENSORS; i++) {
  // ... existing logic applies to all types including ZCLICK
}
```

The only addition: register the Zwift Click button callback in `begin()`:

```cpp
#if BBS_COMPILE_ZWIFT_CLICK
BikeSensorServerSetZwiftClickCallback(staticOnZwiftClickButton);
#endif
```

### 2.6 Button callback integration

```cpp
// BLERelayManager.cpp
void BLERelayManager::onZwiftClickButton(uint32_t buttonMap, uint8_t deviceId) {
  // Option 1: Forward to WebSocket as telemetry (for UI display)
  // Option 2: Map buttons to fan control actions
  // Option 3: Both

  // Example: map LEFT/RIGHT to fan speed adjust
  bool left  = (buttonMap & 0x01) == 0;
  bool right = (buttonMap & 0x04) == 0;
  bool up    = (buttonMap & 0x02) == 0;
  bool down  = (buttonMap & 0x08) == 0;

  if (up)    _fanController->increaseFanSpeed();
  if (down)  _fanController->decreaseFanSpeed();
  // etc.
}
```

### 2.7 API validation

**`ApiActions::validateSensorType()`** (ApiActions.cpp ~line 477):

```cpp
static bool validateSensorType(const char* type) {
  return strcmp(type, "HR") == 0
      || strcmp(type, "CSC") == 0
      || strcmp(type, "ZCLICK") == 0;  // NEW
}
```

### 2.8 WebSocket telemetry

**`SENSORS_SNAPSHOT` frame** (WebSocketTelemetry.cpp):

Already iterates `MAX_SENSORS` and sends each entry. With `MAX_SENSORS = 3`, the 3rd entry (ZCLICK) is automatically included:

```
[count:uint8=3][sensor0:HR:84bytes][sensor1:CSC:84bytes][sensor2:ZCLICK:84bytes]
```

The type byte uses the enum value: `SENSOR_TYPE_ZWIFT_CLICK = 0x08` → sent as-is.

### 2.9 Web UI – app.js

**`DEFAULT_SENSOR_TYPES`** (~line 4556):

```javascript
const DEFAULT_SENSOR_TYPES = [
  { key: "HR",     label: "Heart Rate",   icon: "hr" },
  { key: "CSC",    label: "Speed/Cadence", icon: "csc" },
  { key: "ZCLICK", label: "Zwift Click",  icon: "zclick" },  // NEW
];
```

**`SENSOR_TYPE_REV` mapping** (~line 25):

```javascript
const SENSOR_TYPE_REV = { 0: "HR", 1: "CSC", 2: "ZCLICK" };
// or by enum value:
const SENSOR_TYPE_MAP = { 0x01: "HR", 0x02: "CSC", 0x08: "ZCLICK" };
```

**`getSensorTypeLabel()`** (~line 4586):

```javascript
case "ZCLICK": return "Zwift Click";
```

### 2.10 Web UI – app.css (icon styles)

```css
/* Zwift Click sensor icon */
.sensor-icon-zclick {
  background-image: var(--icon-sensor-zclick-svg-data);
  background-size: contain;
  background-repeat: no-repeat;
}
.sensor-icon-zclick-grayscale {
  background-image: var(--icon-sensor-zclick-svg-data);
  background-size: contain;
  background-repeat: no-repeat;
  filter: grayscale(100%) opacity(0.4);
}
```

### 2.11 Web UI – index.html (SVG icon)

Add a CSS variable for the Zwift Click icon (e.g. a gamepad or click-button icon):

```html
<style>
:root {
  --icon-sensor-zclick-svg-data: url("data:image/svg+xml,...");
}
</style>
```

The actual SVG data should match the existing design language (same size, stroke width, color as HR/CSC icons).

### 2.12 Summary of Homewind changes

| Component | What to add |
|-----------|------------|
| `BLERelayManager.h` | `MAX_SENSORS = 3`, Zwift Click slot index |
| `BLERelayManager.cpp` | Type string mappings, button callback, NVS version bump |
| `NVSSensorData` | `sensors[3]`, version 3 migration |
| `ApiActions.cpp` | Validate `"ZCLICK"` type |
| `WebSocketTelemetry.cpp` | No changes (generic loop handles 3rd sensor) |
| `app.js` | Sensor type definition, type mapping, label |
| `app.css` | Icon classes (connected + grayscale) |
| `index.html` | SVG icon CSS variable |

---

## Implementation Order

### Phase 1: BBS library (foundation)

1. `BBSConfig.h` → compile flag
2. `BluetoothBikeSensorServer.h` → enum + API declarations
3. `SensorRegistry.h` → extend struct (handshake + keep-alive)
4. `SensorDataParser.h/cpp` → parse function + data struct + readVarint
5. `BluetoothBikeSensorServer.cpp` → registry entry + handshake/keepAlive functions
6. `BluetoothBikeSensorServer.cpp` → modify `connectAndSubscribeSensor()` for handshake
7. `BluetoothBikeSensorServer.cpp` → add keep-alive call in loop/tick
8. Scan callbacks → match FC82 / "Zwift Click"
9. **Test:** Standalone BBS test sketch that discovers, connects, and logs button presses

### Phase 2: Homewind integration

1. `BLERelayManager` → type mappings + MAX_SENSORS
2. `NVSSensorData` → extend to 3 sensors + migration
3. `ApiActions` → validate ZCLICK type
4. `BLERelayManager` → button callback + optional fan integration
5. `app.js` → sensor type definition + label + type mapping
6. `app.css` + `index.html` → icon (design provided)
7. **Test:** Discovery → select → NVS save → reboot → auto-reconnect → UI shows connected

---

## Risk areas

| Risk | Mitigation |
|------|-----------|
| Zwift Click requires handshake + keep-alive (unlike HR/CSC) | Extend `SensorTypeConfig` with function pointers; keep existing sensors unaffected |
| NVS schema change (version 2 → 3) | Migration in `loadConfigFromNVS()`: read v2, zero-init 3rd slot, save as v3 |
| Two Zwift Clicks (left + right) | Phase 1: support one Click. Phase 2: extend BBS to support 2 sensors of same type (left/right via device ID) |
| FC82 is 16-bit UUID but chars are 128-bit | Registry already extended with `charUUID128` field |
| Keep-alive must run even when no button is pressed | Add `keepAliveFunc` call in BBS tick/loop, not just in notify callback |
| NimBLE client pool limit (max 3) | BBS already reuses clients; Zwift Click is one additional client |

---

## Reference: Existing sensor integration pattern

To see how HR was integrated end-to-end, check these exact locations:

- **BBS enum:** `BluetoothBikeSensorServer.h:279` → `SENSOR_TYPE_HEART_RATE`
- **BBS registry:** `BluetoothBikeSensorServer.cpp:336` → HR entry in `SENSOR_REGISTRY`
- **BBS parser:** `SensorDataParser.cpp:24` → `parseAndLogHR()`
- **BBS mirror:** `BluetoothBikeSensorServer.cpp:300` → `mirrorHeartRatePacket()`
- **BBS scan:** `BluetoothBikeSensorServer.cpp:369` → `MyAdvertisedDeviceCallbacks::onResult()`
- **BBS connect:** `BluetoothBikeSensorServer.cpp:807` → `connectAndSubscribeSensor()`
- **Homewind type:** `BLERelayManager.cpp:727` → `stringToSensorType("HR")`
- **Homewind NVS:** `BLERelayManager.cpp:1285` → `saveConfigToNVS()`
- **Homewind API:** `ApiActions.cpp:477` → `validateSensorType()`
- **Homewind UI:** `app.js:4556` → `DEFAULT_SENSOR_TYPES`
- **Homewind icon:** `app.css:857` → `.sensor-icon-hr`
