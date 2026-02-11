# ZwiftControlsRelay – Documentation

ESP32 sketch acting as **BLE Central** (connects to one or two Zwift Click controllers) and **BLE Peripheral / Relay** (FC82 server exposing the same controls to tablets or phones). Written for NimBLE on ESP32.

---

## 1. Overview

- **Central role:** Discovers and connects to up to two Zwift Click devices. Left controller = device ID 8 (d-pad / arrows); right controller = device ID 7 (Y/Z/B/A and related). Connection order is fixed: try to connect to the **left (8) first**, then to the **right (7)**. If no left is found within the timeout, the sketch searches for the right only. Device role is determined by manufacturer data (byte 2: 8 or 7) when present; otherwise any device named "Zwift Click" is accepted and the first one is treated as left, the second (different address) as right.
- **Relay role:** A BLE GATT server with service **0xFC82** and manufacturer data set to a neutral value (0xFFFF) so that other centrals (e.g. Zwift app on a tablet) can connect and receive the same Measurement notifications and Control-Point / Response handshake as with physical Clicks.

**Dependencies:** NimBLE (NimBLE-Arduino). Target: ESP32.

---

## 2. Configuration (sketch constants)

| Constant | Default | Description |
|----------|---------|-------------|
| `TARGET_NAME` | `"Zwift Click"` | BLE advertising name used to filter controller devices. |
| `ZWIFT_MFG_ID` | `0x094A` | Zwift, Inc. manufacturer ID (little-endian in advertising). |
| `DEVICE_ID_LEFT` | `8` | Left controller device ID in manufacturer data (arrows). |
| `DEVICE_ID_RIGHT` | `7` | Right controller device ID (Y/Z/B/A, etc.). |
| `SCAN_PHASE_TIMEOUT_MS` | `8000` | Timeout per scan phase (ms). After this time without a suitable device, the state machine advances (e.g. give up on left and search for right, or rescan). |
| `KEEPALIVE_INTERVAL_MS` | `5000` | Interval for sending Info-Request (`0x00 0x08 0x00`) to the Control characteristic. Set to `0` to disable. |
| `RIDEON_KEEPALIVE_MS` | `30000` | Interval for sending "RideOn" to the Control characteristic. Set to `0` to disable. |

Without keep-alive, Zwift Clicks typically enter standby after about one minute (LEDs off) and stop sending notifications.

**Relay advertising:**  
- Default: Manufacturer data `0xFF 0xFF 0x00 0x00 0x00` (neutral; does not impersonate Zwift).  
- Optional (commented out): Zwift-style data `0x4A 0x09 0x0B 0xEA 0x77`.

---

## 3. Central connection flow and state machine

Connection logic is driven by `DualState` (enum, 0–4) and timers. Only one scan runs at a time; it is stopped when a device is chosen for connection or when the phase times out.

### 3.1 State machine (DualState)

- **ST_SCAN_LEFT (0)** – Scanning for left (8).  
  - **Device found:** Address stored in `addrLeft` (no heap copy), scan stopped, `doConnectLeft = true`, state → `ST_CONN_LEFT`.  
  - **Timeout:** "Kein linker (8) – suche rechten (7)…", state → `ST_SCAN_RIGHT`, new scan.

- **ST_CONN_LEFT (1)** – Connecting to left (8) via `connectController()`.  
  - **Success:** "Linker (8) bereit. Suche rechten (7)…", state → `ST_SCAN_RIGHT`, scan started.  
  - **Failure:** `connectController` deletes the client internally, state → `ST_SCAN_LEFT`, rescan.

- **ST_SCAN_RIGHT (2)** – Scanning for right (7).  
  - **Device found:** Address stored in `addrRight`, scan stopped, `doConnectRight = true`, state → `ST_CONN_RIGHT`.  
  - **Timeout with left connected:** "Rechter (7) nicht gefunden – nur linker (8) aktiv.", state → `ST_RUNNING`.  
  - **Timeout with no controller:** "Keiner gefunden. Erneuter Scan…", state → `ST_SCAN_LEFT`, rescan (loop).

- **ST_CONN_RIGHT (3)** – Connecting to right (7).  
  - **Success:** "Beide verbunden…" or "Nur rechter (7) verbunden…", state → `ST_RUNNING`.  
  - **Failure:** `cleanupAndRescan()` (deletes both clients), state → `ST_SCAN_LEFT`.

- **ST_RUNNING (4)** – Operational. Keep-alive runs. On disconnect → `needRescan = true`, handled in loop → `cleanupAndRescan()`.

### 3.2 Scanning behaviour

Scanning is **not** done only once. It is started/stopped **multiple times**:
1. At startup (phase 0: left).
2. After "no left" timeout (phase 2: right).
3. After left connected (phase 2: right).
4. After "neither found" timeout (back to phase 0).
5. After disconnect or connection failure (back to phase 0).

### 3.3 Device selection in scan callback

- Only devices with name equal to `TARGET_NAME` are considered.
- Manufacturer data byte 2 gives device ID (8=left, 7=right). If missing (devId==0), the device is accepted as either.
- In `ST_SCAN_RIGHT`: devices with the same address as `addrLeft` are skipped.

---

## 4. Per-controller handshake (setupSession)

After BLE connection, `connectController()` calls `discoverChars()` and `setupSession()`:

1. Resolve service: FC82 first, then legacy UUID.
2. Pick characteristics: Measurement (Notify), Control (Write), Response (Indicate) – by UUID first, then by property fallback.
3. Subscribe to Response (Indicate).
4. Write "RideOn" (6 bytes) to Control, wait 400 ms.
5. Write Info-Request `0x00 0x08 0x00`, wait 200 ms.
6. Subscribe to Measurement (Notify).
7. "Controller Ready."

On any failure, the client is immediately deleted via `NimBLEDevice::deleteClient()` inside `connectController()`.

---

## 5. Relay server (peripheral)

- **Service:** 0xFC82.
- **Characteristics:**
  - Measurement (Notify): forwarded raw payloads from physical Clicks.
  - Control (Write/WriteNR): receives RideOn, Info-Request, etc.
  - Response (Indicate+Read): answers RideOn with RideOn indication.

**Control-Point write:** Single-line Serial output: `CP <length>` or `CP <length> RideOn`. Uses `containsBytes()` (no heap `std::string`) to detect "RideOn".

**Advertising:** FC82, Service Data, Mfg 0xFFFF, Scan Response "Zwift Relay".

---

## 6. Keep-alive (ST_RUNNING only)

- Every `KEEPALIVE_INTERVAL_MS` (5 s): Info-Request `0x00 0x08 0x00` to both controllers.
- Every `RIDEON_KEEPALIVE_MS` (30 s): "RideOn" to both controllers.
- Either can be disabled by setting its constant to `0`.

---

## 7. Measurement / keypress (0x23)

1. Raw data forwarded to relay via `relayMeas->notify()`.
2. Byte 0 == 0x23: parse protobuf varint field 1 for button map; fallback raw LE32 at bytes 1–4.
3. Pressed = bit low (inverted logic).
4. `printPressedButtons()`: builds complete line in stack buffer, prints via single `Serial.println()` protected by FreeRTOS mutex. Prints **nothing** if no button is pressed.

---

## 8. Button names and bit masks

| Mask (hex) | Name      | Mask (hex) | Name       |
|------------|-----------|------------|------------|
| 0x00000001 | LEFT      | 0x00000010 | A          |
| 0x00000002 | UP        | 0x00000020 | B          |
| 0x00000004 | RIGHT     | 0x00000040 | Y          |
| 0x00000008 | DOWN      | 0x00000100 | Z          |
| 0x00000200 | SHFT_UP_L | 0x00002000 | SHFT_UP_R  |
| 0x00000400 | SHFT_DN_L | 0x00004000 | SHFT_DN_R  |
| 0x00000800 | POWERUP_L | 0x00010000 | POWERUP_R  |
| 0x00001000 | ONOFF_L   | 0x00020000 | ONOFF_R    |

---

## 9. Heap safety

The following heap problems were identified in the original code and fixed:

### Leak 1: `new DualClientCallbacks()` on every connection attempt
**Problem:** Each call to `setClientCallbacks(new DualClientCallbacks(), false)` allocated a callback object that was never freed (`false` = NimBLE won't delete it; no manual `delete` existed). Every retry leaked ~16 bytes.  
**Fix:** Single static instance `clientCb`. Passed by pointer with `false` (correct: static must not be deleted).

### Leak 2: NimBLE clients not deleted in onDisconnect
**Problem:** `pClientLeft->disconnect()` / `pClientRight->disconnect()` only disconnected but did not call `NimBLEDevice::deleteClient()`. The client objects stayed in NimBLE's internal pool (max 3 slots). After 1–2 disconnect/reconnect cycles, `createClient()` would fail.  
**Fix:** `cleanupAndRescan()` uses `NimBLEDevice::deleteClient()` for both clients, freeing pool slots and heap.

### Leak 3: `new NimBLEAdvertisedDevice(*)` heap copy
**Problem:** Scan callback copied the entire `NimBLEAdvertisedDevice` (~200+ bytes) to heap. After successful connection, the copy remained allocated until disconnect.  
**Fix:** Only the `NimBLEAddress` (8 bytes, stack/static) is stored. No heap allocation.

### Leak 4: `new MyScanCallbacks()`, `new RelayCtrlCallbacks()`
**Problem:** Allocated once and never freed. Not a repeated leak but unnecessary heap use.  
**Fix:** Static instances `scanCb`, `relayCtrlCb`.

### Problem 5: disconnect() called on other client from inside BLE callback
**Problem:** `onDisconnect` called `pClientLeft->disconnect()` / `pClientRight->disconnect()` directly. This ran inside the NimBLE host task, risking deadlock or stack corruption.  
**Fix:** Callback only sets `needRescan = true`. All cleanup runs in `loop()` via `cleanupAndRescan()`.

### Problem 6: shared temporary globals
**Problem:** `pRcSvc`, `pMeas`, `pCtrl`, `pResp`, `pClient`, `doConnectDual` were shared between left and right connection code. Error-prone.  
**Fix:** `connectController()` uses local variables and out-parameters. Shared temp globals removed.

### Heap monitoring
`setup()` and end of successful connection print `ESP.getFreeHeap()` so heap health can be verified on Serial.

---

## 10. Serial output summary

| Event | Output |
|-------|--------|
| Boot | "ZwiftControlsRelay – …" + "Free heap: N bytes" |
| Relay up | "📡 Relay aktiv (FC82, Mfg 0xFFFF)…" |
| Device found | "Gefunden: name addr (RSSI: N)" |
| Left selected | "📱 Linker (8) addr – verbinde." |
| Right selected | "📱 Rechter (7) addr – verbinde." |
| Connecting | "🔗 Verbinde mit linkem/rechtem …" |
| Handshake OK | "✅ Controller Ready." |
| Left ready | "✅ Linker (8) bereit. Suche rechten (7)…" |
| Both connected | "✅ Beide verbunden. [L]=Pfeile, [R]=YZBA+. Relay aktiv." + heap |
| Only right | "✅ Nur rechter (7) verbunden. [R]=YZBA+. Relay aktiv." + heap |
| No left timeout | "⏱️ Kein linker (8) – suche rechten (7)…" |
| No right, left OK | "⏱️ Rechter (7) nicht gefunden – nur linker (8) aktiv." |
| Neither found | "⏱️ Keiner gefunden. Erneuter Scan…" |
| Disconnect | "❌ Controller getrennt (Code: N). Rescan…" |
| Errors | "❌ label: Verbindung/Service/Handshake fehlgeschlagen." |
| Relay CP write | "CP N" or "CP N RideOn" |
| Keypress | "[L] KEY: LEFT" or "[R] KEY: A, B" (only when pressed) |

---

## 11. Quick reference (for humans and AI)

- **Two Clicks:** Central tries left (8) first, then right (7). Timeout 8 s per phase; if neither found, scan restarts.
- **Relay:** FC82, Mfg 0xFFFF. Measurement forwarded; CP writes logged as one line.
- **Keep-alive:** Info-Request 5 s, RideOn 30 s (configurable; 0 = off).
- **Heap safe:** All callbacks are static; clients freed via `deleteClient()`; device addresses stored as `NimBLEAddress` (no `new`); disconnect cleanup runs in `loop()` not in BLE callback.
- **Serial:** Keypress only when pressed, one line + mutex; heap printed at boot and after connection.
