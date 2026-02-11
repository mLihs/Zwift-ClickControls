/*
 * ZwiftControlsRelay
 *
 * Central: connects to both Zwift Click controllers (L + R).
 * Peripheral (Relay): BLE server with service FC82 – exposes the
 * controllers' Measurement/Control/Response to other centrals (e.g. tablet).
 *
 * Requires: NimBLE (NimBLE-Arduino), ESP32
 *
 * Heap-safety notes:
 *   - All BLE callbacks are static instances (no `new` per connection).
 *   - Clients are deleted via NimBLEDevice::deleteClient() (frees pool slot + heap).
 *   - NimBLEAdvertisedDevice copies avoided; only NimBLEAddress is stored.
 *   - Disconnect handler only sets a flag; cleanup runs safely in loop().
 */

#include <NimBLEDevice.h>
#include <freertos/semphr.h>

// ═════════════════════════════════════════════
// Configuration
// ═════════════════════════════════════════════
static const std::string TARGET_NAME        = "Zwift Click";
static const uint16_t ZWIFT_MFG_ID          = 0x094A;
static const uint8_t  DEVICE_ID_LEFT        = 8;   // d-pad / arrows
static const uint8_t  DEVICE_ID_RIGHT       = 7;   // Y Z B A + modifiers
static const uint32_t SCAN_PHASE_TIMEOUT_MS = 8000;
static const uint32_t KEEPALIVE_INTERVAL_MS = 5000; // Info-Request; 0 = off
static const uint32_t RIDEON_KEEPALIVE_MS   = 30000; // "RideOn";   0 = off

// Relay advertising: neutral manufacturer data (0xFFFF)
static const uint8_t RELAY_MFG_DATA[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00 };
// Zwift-style alternative (commented out):
// static const uint8_t RELAY_MFG_DATA[] = { 0x4A, 0x09, 0x0B, 0xEA, 0x77 };

// BLE UUIDs
static const NimBLEUUID SVC_FC82((uint16_t)0xFC82);
static const NimBLEUUID SVC_OLD("00000001-19ca-4651-86e5-fa29dcdd09d1");
static const NimBLEUUID CHR_MEAS("00000002-19ca-4651-86e5-fa29dcdd09d1");
static const NimBLEUUID CHR_CTRL("00000003-19ca-4651-86e5-fa29dcdd09d1");
static const NimBLEUUID CHR_RESP("00000004-19ca-4651-86e5-fa29dcdd09d1");

// ═════════════════════════════════════════════
// State machine
// ═════════════════════════════════════════════
enum DualState : uint8_t {
  ST_SCAN_LEFT  = 0,
  ST_CONN_LEFT  = 1,
  ST_SCAN_RIGHT = 2,
  ST_CONN_RIGHT = 3,
  ST_RUNNING    = 4,
};

// Central – controller connections (no raw new/delete; NimBLE owns clients)
static NimBLEAddress  addrLeft;             // stored from scan; no heap alloc
static NimBLEAddress  addrRight;
static NimBLEClient*  clientLeft    = nullptr;
static NimBLEClient*  clientRight   = nullptr;
static NimBLERemoteCharacteristic* measLeft  = nullptr;
static NimBLERemoteCharacteristic* measRight = nullptr;
static NimBLERemoteCharacteristic* ctrlLeft  = nullptr;
static NimBLERemoteCharacteristic* ctrlRight = nullptr;

static volatile DualState dualState      = ST_SCAN_LEFT;
static volatile bool      needRescan     = false;  // set from onDisconnect callback
static volatile bool      doConnectLeft  = false;
static volatile bool      doConnectRight = false;
static uint32_t           scanStartTime  = 0;
static uint32_t           lastKeepalive  = 0;
static uint32_t           lastRideOn     = 0;

// Relay server (created once, never freed)
static NimBLEServer*         relayServer = nullptr;
static NimBLECharacteristic* relayMeas   = nullptr;
static NimBLECharacteristic* relayCtrl   = nullptr;
static NimBLECharacteristic* relayResp   = nullptr;

// Serial mutex for atomic line output from BLE callbacks
static SemaphoreHandle_t serialMux = nullptr;

// ═════════════════════════════════════════════
// Protobuf varint helper
// ═════════════════════════════════════════════
static bool readVarint(const uint8_t* d, size_t len, size_t& i, uint32_t& out) {
  out = 0;
  for (uint32_t shift = 0; i < len && shift <= 28; shift += 7) {
    uint8_t b = d[i++];
    out |= (uint32_t)(b & 0x7F) << shift;
    if (!(b & 0x80)) return true;
  }
  return false;
}

// ═════════════════════════════════════════════
// Button table
// ═════════════════════════════════════════════
struct Btn { uint32_t mask; const char* name; };
static const Btn kButtons[] = {
  {0x00000001,"LEFT"}, {0x00000002,"UP"}, {0x00000004,"RIGHT"}, {0x00000008,"DOWN"},
  {0x00000010,"A"},    {0x00000020,"B"},  {0x00000040,"Y"},     {0x00000100,"Z"},
  {0x00000200,"SHFT_UP_L"}, {0x00000400,"SHFT_DN_L"},
  {0x00000800,"POWERUP_L"}, {0x00001000,"ONOFF_L"},
  {0x00002000,"SHFT_UP_R"}, {0x00004000,"SHFT_DN_R"},
  {0x00010000,"POWERUP_R"}, {0x00020000,"ONOFF_R"},
};
static constexpr size_t kNumButtons = sizeof(kButtons) / sizeof(kButtons[0]);

// Print pressed buttons in one atomic Serial.println (mutex-protected).
// Returns silently if no button is pressed (avoids "(none)" flood).
static void printPressedButtons(uint32_t bmap, const char* prefix) {
  char buf[80];
  char* q = buf;
  int cap = sizeof(buf);
  q += snprintf(q, cap, "%sKEY: ", prefix);
  bool first = true;
  for (size_t i = 0; i < kNumButtons && (q - buf) < cap - 12; i++) {
    if ((bmap & kButtons[i].mask) == 0) {
      if (!first) q += snprintf(q, cap - (q - buf), ", ");
      q += snprintf(q, cap - (q - buf), "%s", kButtons[i].name);
      first = false;
    }
  }
  if (first) return;
  if (xSemaphoreTake(serialMux, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println(buf);
    xSemaphoreGive(serialMux);
  }
}

// Check if byte sequence contains a substring (avoids std::string heap alloc)
static bool containsBytes(const uint8_t* hay, size_t hlen, const char* needle, size_t nlen) {
  if (hlen < nlen) return false;
  for (size_t i = 0; i <= hlen - nlen; i++)
    if (memcmp(hay + i, needle, nlen) == 0) return true;
  return false;
}

// ═════════════════════════════════════════════
// Relay: Control-Point write callback
// STATIC instance – no heap allocation per connection.
// ═════════════════════════════════════════════
static class RelayCtrlCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
    NimBLEAttValue val = chr->getValue();
    size_t len = val.length();
    bool isRideOn = (relayResp && len >= 6
                     && containsBytes(val.data(), len, "RideOn", 6));
    if (isRideOn) {
      relayResp->setValue((uint8_t*)"RideOn", 6);
      relayResp->indicate();
    }
    char line[32];
    snprintf(line, sizeof(line), "CP %u%s", (unsigned)len, isRideOn ? " RideOn" : "");
    Serial.println(line);
  }
} relayCtrlCb;

// ═════════════════════════════════════════════
// Central: Measurement notify → Serial + Relay
// ═════════════════════════════════════════════
static void onMeasNotify(NimBLERemoteCharacteristic* chr,
                         uint8_t* data, size_t len, bool) {
  if (len == 0) return;
  // Forward to relay subscribers
  if (relayMeas) { relayMeas->setValue(data, len); relayMeas->notify(); }
  if (data[0] != 0x23) return;

  // Parse button map (protobuf field 1 varint; fallback raw LE32)
  uint32_t bmap = 0xFFFFFFFF;
  size_t idx = 1;
  bool ok = false;
  while (idx < len) {
    uint32_t tag;
    if (!readVarint(data, len, idx, tag)) break;
    if ((tag >> 3) == 1 && (tag & 7) == 0) { ok = readVarint(data, len, idx, bmap); break; }
    if ((tag & 7) == 0) { uint32_t t; if (!readVarint(data, len, idx, t)) break; }
    else if ((tag & 7) == 2) { uint32_t l; if (!readVarint(data, len, idx, l)) break; idx += l; if (idx > len) break; }
    else break;
  }
  if (!ok && len >= 5)
    bmap = (uint32_t)data[1] | ((uint32_t)data[2]<<8) | ((uint32_t)data[3]<<16) | ((uint32_t)data[4]<<24);

  const char* pfx = (measLeft && chr == measLeft)   ? "[L] "
                   : (measRight && chr == measRight) ? "[R] " : "";
  printPressedButtons(bmap, pfx);
}

static void onRespIndicate(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  Serial.print("⬅️ INDICATE: ");
  for (size_t i = 0; i < len; i++) Serial.write(data[i]);
  Serial.println();
}

// ═════════════════════════════════════════════
// Central: Client callbacks
// STATIC instance – FIX for Leak #1 (was `new DualClientCallbacks()` per connect).
// The second arg `false` in setClientCallbacks means "don't delete" which is
// correct for a static instance. Previously a new instance was allocated and
// never freed on each connection attempt.
// ═════════════════════════════════════════════
static class ClientCb : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*, int reason) override {
    Serial.printf("❌ Controller getrennt (Code: %d). Rescan…\n", reason);
    // FIX for Problem #5: don't call disconnect()/deleteClient() from inside
    // the BLE host task callback. Just set a flag; loop() handles cleanup.
    needRescan = true;
  }
} clientCb;

// ═════════════════════════════════════════════
// Central: cleanup + restart scan
// FIX for Leak #2: uses deleteClient() (frees NimBLE pool slot + heap),
// not just disconnect() which left the client object alive in the pool.
// ═════════════════════════════════════════════
static void cleanupAndRescan() {
  needRescan = false;  // prevent re-entry
  NimBLEDevice::getScan()->stop();

  if (clientLeft)  { NimBLEDevice::deleteClient(clientLeft);  clientLeft  = nullptr; }
  if (clientRight) { NimBLEDevice::deleteClient(clientRight); clientRight = nullptr; }
  measLeft = measRight = ctrlLeft = ctrlRight = nullptr;
  addrLeft  = NimBLEAddress();
  addrRight = NimBLEAddress();

  dualState = ST_SCAN_LEFT;
  scanStartTime = millis();
  NimBLEDevice::getScan()->start(0, false);
}

// ═════════════════════════════════════════════
// Central: discover service + characteristics
// Uses local out-parameters instead of shared globals (was pMeas/pCtrl/pResp).
// ═════════════════════════════════════════════
static bool discoverChars(NimBLEClient* c,
                          NimBLERemoteCharacteristic*& outMeas,
                          NimBLERemoteCharacteristic*& outCtrl,
                          NimBLERemoteCharacteristic*& outResp) {
  NimBLERemoteService* svc = c->getService(SVC_FC82);
  if (!svc) svc = c->getService(SVC_OLD);
  if (!svc) return false;

  outMeas = svc->getCharacteristic(CHR_MEAS);
  outCtrl = svc->getCharacteristic(CHR_CTRL);
  outResp = svc->getCharacteristic(CHR_RESP);
  if (outMeas && outCtrl && outResp) return true;

  // Fallback: match by property
  outMeas = outCtrl = outResp = nullptr;
  for (auto* ch : svc->getCharacteristics(true)) {
    if (!outMeas && ch->canNotify())                              outMeas = ch;
    if (!outCtrl && (ch->canWrite() || ch->canWriteNoResponse())) outCtrl = ch;
    if (!outResp && ch->canIndicate())                            outResp = ch;
  }
  return (outMeas && outCtrl && outResp);
}

// ═════════════════════════════════════════════
// Central: session handshake (RideOn + subscribe)
// ═════════════════════════════════════════════
static bool setupSession(NimBLERemoteCharacteristic* meas,
                         NimBLERemoteCharacteristic* ctrl,
                         NimBLERemoteCharacteristic* resp) {
  if (resp->canIndicate()) resp->subscribe(true, onRespIndicate);
  if (!ctrl->writeValue((uint8_t*)"RideOn", 6, true)) return false;
  delay(400);
  static const uint8_t infoReq[] = { 0x00, 0x08, 0x00 };
  ctrl->writeValue(infoReq, sizeof(infoReq), true);
  delay(200);
  if (!meas->subscribe(true, onMeasNotify)) return false;
  lastKeepalive = lastRideOn = millis();
  Serial.println("✅ Controller Ready.");
  return true;
}

// ═════════════════════════════════════════════
// Central: connect to one controller
// Returns client on success, nullptr on failure.
// On failure: client is deleteClient()'d immediately (no leak).
// FIX for Leak #1 + #2: uses static clientCb and always deleteClient on error.
// ═════════════════════════════════════════════
static NimBLEClient* connectController(const NimBLEAddress& addr,
                                       const char* label,
                                       NimBLERemoteCharacteristic*& outMeas,
                                       NimBLERemoteCharacteristic*& outCtrl) {
  NimBLEClient* c = NimBLEDevice::createClient();
  c->setConnectionParams(12, 24, 0, 200);
  c->setClientCallbacks(&clientCb, false);  // static instance, don't delete

  if (!c->connect(addr)) {
    Serial.printf("❌ %s: Verbindung fehlgeschlagen.\n", label);
    NimBLEDevice::deleteClient(c);
    return nullptr;
  }

  NimBLERemoteCharacteristic* resp = nullptr;
  if (!discoverChars(c, outMeas, outCtrl, resp)) {
    Serial.printf("❌ %s: Service/Chars fehlgeschlagen.\n", label);
    NimBLEDevice::deleteClient(c);
    return nullptr;
  }

  if (!setupSession(outMeas, outCtrl, resp)) {
    Serial.printf("❌ %s: Handshake fehlgeschlagen.\n", label);
    NimBLEDevice::deleteClient(c);
    return nullptr;
  }

  return c;
}

// ═════════════════════════════════════════════
// Keep-alive helpers
// ═════════════════════════════════════════════
static void sendKeepalive(NimBLERemoteCharacteristic* ctrl) {
  if (!ctrl) return;
  static const uint8_t req[] = { 0x00, 0x08, 0x00 };
  ctrl->writeValue(req, sizeof(req), true);
}

static void sendRideOn(NimBLERemoteCharacteristic* ctrl) {
  if (!ctrl) return;
  ctrl->writeValue((uint8_t*)"RideOn", 6, true);
}

// ═════════════════════════════════════════════
// Scan callback
// STATIC instance – FIX for Leak #4 (was `new MyScanCallbacks()`).
// FIX for Leak #3: stores only NimBLEAddress (8 bytes) instead of
// `new NimBLEAdvertisedDevice(*)` (~200+ bytes heap copy).
// ═════════════════════════════════════════════
static class ScanCb : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string name = dev->getName();
    if (!name.empty())
      Serial.printf("Gefunden: %s %s (RSSI: %d)\n",
                    name.c_str(), dev->getAddress().toString().c_str(),
                    dev->getRSSI());

    if (name != TARGET_NAME) return;

    // Parse manufacturer data for device ID (8=left, 7=right)
    uint8_t devId = 0;
    if (dev->getManufacturerDataCount() > 0) {
      std::string mfg = dev->getManufacturerData(0);
      if (mfg.length() >= 3) {
        uint16_t id = (uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
        if (id == ZWIFT_MFG_ID) devId = (uint8_t)mfg[2];
      }
    }

    bool canLeft = (devId == DEVICE_ID_LEFT) || (devId == 0);

    if (dualState == ST_SCAN_LEFT && canLeft) {
      Serial.printf("📱 Linker (8) %s – verbinde.\n",
                    dev->getAddress().toString().c_str());
      addrLeft = dev->getAddress();
      NimBLEDevice::getScan()->stop();
      doConnectLeft = true;
      dualState = ST_CONN_LEFT;

    } else if (dualState == ST_SCAN_RIGHT) {
      // Skip if same address as left (avoid reusing same device)
      if (addrLeft == dev->getAddress()) return;
      Serial.printf("📱 Rechter (7) %s – verbinde.\n",
                    dev->getAddress().toString().c_str());
      addrRight = dev->getAddress();
      NimBLEDevice::getScan()->stop();
      doConnectRight = true;
      dualState = ST_CONN_RIGHT;
    }
  }
} scanCb;

// ═════════════════════════════════════════════
// Relay Server setup (called once)
// ═════════════════════════════════════════════
static void setupRelayServer() {
  relayServer = NimBLEDevice::createServer();

  NimBLEService* svc = relayServer->createService(SVC_FC82);
  relayMeas = svc->createCharacteristic(CHR_MEAS, NIMBLE_PROPERTY::NOTIFY);
  relayCtrl = svc->createCharacteristic(CHR_CTRL, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  relayResp = svc->createCharacteristic(CHR_RESP, NIMBLE_PROPERTY::INDICATE | NIMBLE_PROPERTY::READ);
  relayCtrl->setCallbacks(&relayCtrlCb);  // static instance
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad;
  ad.setManufacturerData(RELAY_MFG_DATA, sizeof(RELAY_MFG_DATA));
  ad.addServiceUUID(SVC_FC82);
  static const uint8_t svcPayload[] = { 0x0B, 0xEA, 0x77 };
  ad.setServiceData(SVC_FC82, std::string((const char*)svcPayload, 3));
  adv->setAdvertisementData(ad);

  NimBLEAdvertisementData sr;
  sr.setName("Zwift Relay");
  adv->setScanResponseData(sr);
  adv->start(0);

  Serial.println("📡 Relay aktiv (FC82, Mfg 0xFFFF). Bereit für Central-Geräte.");
}

// ═════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("ZwiftControlsRelay – Central (2 Clicks) + BLE Relay (FC82)");
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

  serialMux = xSemaphoreCreateMutex();

  NimBLEDevice::init("ZwiftRelay");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  setupRelayServer();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCb);  // static instance
  scan->setActiveScan(true);
  scan->setInterval(97);
  scan->setWindow(67);
  scanStartTime = millis();
  scan->start(0, false);
}

// ═════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════
void loop() {
  // ── Handle disconnect (flag set from BLE callback; safe cleanup here) ──
  if (needRescan) {
    cleanupAndRescan();
    delay(200);
    return;
  }

  // ── Keep-alive (running state) ──
  if (dualState == ST_RUNNING) {
    uint32_t now = millis();
    if (KEEPALIVE_INTERVAL_MS && (now - lastKeepalive >= KEEPALIVE_INTERVAL_MS)) {
      lastKeepalive = now;
      sendKeepalive(ctrlLeft);
      sendKeepalive(ctrlRight);
    }
    if (RIDEON_KEEPALIVE_MS && (now - lastRideOn >= RIDEON_KEEPALIVE_MS)) {
      lastRideOn = now;
      sendRideOn(ctrlLeft);
      sendRideOn(ctrlRight);
    }
    delay(200);
    return;
  }

  // ── Scan timeout: left phase ──
  if (dualState == ST_SCAN_LEFT && (millis() - scanStartTime >= SCAN_PHASE_TIMEOUT_MS)) {
    Serial.println("⏱️ Kein linker (8) – suche rechten (7)…");
    NimBLEDevice::getScan()->stop();
    dualState = ST_SCAN_RIGHT;
    delay(100);
    scanStartTime = millis();
    NimBLEDevice::getScan()->start(0, false);
  }

  // ── Scan timeout: right phase ──
  if (dualState == ST_SCAN_RIGHT && (millis() - scanStartTime >= SCAN_PHASE_TIMEOUT_MS)) {
    NimBLEDevice::getScan()->stop();
    if (clientLeft) {
      Serial.println("⏱️ Rechter (7) nicht gefunden – nur linker (8) aktiv.");
      dualState = ST_RUNNING;
    } else {
      Serial.println("⏱️ Keiner gefunden. Erneuter Scan…");
      dualState = ST_SCAN_LEFT;
      scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
    }
  }

  // ── Connect left ──
  if (doConnectLeft && dualState == ST_CONN_LEFT) {
    doConnectLeft = false;
    Serial.println("🔗 Verbinde mit linkem (8)…");

    NimBLERemoteCharacteristic* m = nullptr;
    NimBLERemoteCharacteristic* ct = nullptr;
    clientLeft = connectController(addrLeft, "Linker (8)", m, ct);

    if (!clientLeft) {
      addrLeft = NimBLEAddress();
      dualState = ST_SCAN_LEFT;
      scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    measLeft = m;
    ctrlLeft = ct;
    Serial.println("✅ Linker (8) bereit. Suche rechten (7)…");
    dualState = ST_SCAN_RIGHT;
    scanStartTime = millis();
    NimBLEDevice::getScan()->start(0, false);
    delay(200);
    return;
  }

  // ── Connect right ──
  if (doConnectRight && dualState == ST_CONN_RIGHT) {
    doConnectRight = false;
    Serial.println("🔗 Verbinde mit rechtem (7)…");

    NimBLERemoteCharacteristic* m = nullptr;
    NimBLERemoteCharacteristic* ct = nullptr;
    clientRight = connectController(addrRight, "Rechter (7)", m, ct);

    if (!clientRight) {
      // Full cleanup on right failure
      cleanupAndRescan();
      delay(200);
      return;
    }
    measRight = m;
    ctrlRight = ct;
    Serial.println(clientLeft
      ? "✅ Beide verbunden. [L]=Pfeile, [R]=YZBA+. Relay aktiv."
      : "✅ Nur rechter (7) verbunden. [R]=YZBA+. Relay aktiv.");
    dualState = ST_RUNNING;
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    delay(200);
    return;
  }

  delay(200);
}
