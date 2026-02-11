#include <NimBLEDevice.h>

// =============================
// CONFIG
// =============================
// Zwift Click: Linker (8) = Pfeile, Rechter (7) = YZBA+.
// USE_DUAL_CONNECTION 1 = mit BEIDEN verbinden, Tasten als [L]/[R] ausgeben.
// 0 = wie bisher: nur einer (zuerst 8, Fallback 7, Timeout wenn 8 keine Events).
static std::string targetName = "Zwift Click";   // Ride/SF2: "Zwift SF2"
static const uint16_t ZWIFT_MFG_ID = 0x094A;    // Zwift manufacturer ID
static const uint8_t ZWIFT_DEVICE_LEFT = 8;      // Links (Pfeile)
static const uint8_t ZWIFT_DEVICE_RIGHT = 7;     // Rechts (YZBA+)
static const uint32_t SCAN_WAIT_MS = 5000;       // Einzelmodus: warten auf anderen
static const uint32_t SCAN_DUAL_WAIT_MS = 8000;  // Dual: so lange auf beide 7+8 warten
static const uint32_t NO_EVENT_TIMEOUT_MS = 8000; // Einzelmodus: 8 ohne Events → zu 7 wechseln
// Keep-Alive: periodisch Info-Request 00 08 00 an Control Point. Wichtig: Clicks gehen nach ~1 min
// in Standby (Lichter aus) und senden dann keine Notifications mehr – Intervall deutlich unter 1 min.
static const uint32_t KEEPALIVE_INTERVAL_MS = 5000;   // alle 5 s (bei 15 s schlafen Clicks ein)
// Zusätzlich alle 30 s „RideOn“ senden (wie App?), hält Session evtl. wach. 0 = aus.
static const uint32_t RIDEON_KEEPALIVE_MS = 30000;

#define USE_DUAL_CONNECTION  1

// Logging: 0 = nur Tasten + kurze EVT-Zeile, 1 = jede Notification als Hex (Debug)
#define DEBUG_NOTIFY_RAW  0
#if DEBUG_NOTIFY_RAW
  #define DEBUG_NOTIFY_MAX_BYTES  48
#endif
// Kompakte Zeile pro Tastendruck (z.B. "KEY: LEFT" oder "KEY: A,UP")
#define LOG_COMPACT  1
// Jede eingehende Notification kurz bestätigen (msgId), damit man sieht ob überhaupt was ankommt
#define LOG_EVT_ALWAYS  1

// Service UUIDs: old vs new (Jan 2025 change) :contentReference[oaicite:5]{index=5}
static const NimBLEUUID RC_SERVICE_OLD("00000001-19ca-4651-86e5-fa29dcdd09d1");
static const NimBLEUUID RC_SERVICE_NEW((uint16_t)0xFC82);

// Known characteristic UUIDs for the *old* RC service (Play article) :contentReference[oaicite:6]{index=6}
static const NimBLEUUID RC_MEAS_CHAR_OLD("00000002-19ca-4651-86e5-fa29dcdd09d1"); // Notify
static const NimBLEUUID RC_CTRL_CHAR_OLD("00000003-19ca-4651-86e5-fa29dcdd09d1"); // Write
static const NimBLEUUID RC_RESP_CHAR_OLD("00000004-19ca-4651-86e5-fa29dcdd09d1"); // Indicate/Read

// Some firmwares might keep 128-bit chars even with FC82 service. If not, we'll auto-detect by properties.

// =============================
// GLOBALS
// =============================
NimBLEAdvertisedDevice* pDevice = nullptr;
NimBLEClient* pClient = nullptr;
bool doConnect = false;

// Scan: zuerst auf 8 warten, sonst 7; nach "8 ohne Events" nur noch 7
static NimBLEAdvertisedDevice* pDeviceCandidate = nullptr;
static uint32_t scanStartTime = 0;
static bool onlyRightNextScan = false;   // true = einmal 8 ohne Events gehabt → nur noch 7

// Nach Verbindung: bei 8 prüfen ob Events kommen, sonst auf 7 wechseln
static uint8_t connectedDeviceId = 0;    // 7 oder 8
static uint32_t connectionTime = 0;      // wann setupRideSession fertig
static uint32_t lastKeypressTime = 0;    // letzter 0x23 Empfang

// Dual-Modus: zuerst LINKEN (8) verbinden + Handshake („Signal“ an rechten), dann RECHTEN (7) suchen und verbinden
static NimBLEAdvertisedDevice* pDeviceLeft = nullptr;
static NimBLEAdvertisedDevice* pDeviceRight = nullptr;
static NimBLEClient* pClientLeft = nullptr;
static NimBLEClient* pClientRight = nullptr;
static NimBLERemoteCharacteristic* pDualLeftMeas = nullptr;
static NimBLERemoteCharacteristic* pDualRightMeas = nullptr;
static NimBLERemoteCharacteristic* pDualLeftCtrl = nullptr;
static NimBLERemoteCharacteristic* pDualRightCtrl = nullptr;
static bool doConnectDual = false;
static uint32_t lastKeepaliveTime = 0;
static uint32_t lastRideOnKeepaliveTime = 0;
static uint8_t dualState = 0;  // 0=idle, 1=connect left, 2=left ok, scan for right, 3=connect right, 4=both ok

NimBLERemoteService* pRcSvc = nullptr;
NimBLERemoteCharacteristic* pMeas = nullptr; // Notify
NimBLERemoteCharacteristic* pCtrl = nullptr; // Write
NimBLERemoteCharacteristic* pResp = nullptr; // Indicate/Read


// =============================
// Helpers: protobuf varint decode
// =============================
static bool readVarint(const uint8_t* data, size_t len, size_t& i, uint32_t& out) {
  out = 0;
  uint32_t shift = 0;
  while (i < len && shift <= 28) {
    uint8_t b = data[i++];
    out |= (uint32_t)(b & 0x7F) << shift;
    if ((b & 0x80) == 0) return true;
    shift += 7;
  }
  return false;
}

// Button-Masken nach Makinolo Zwift Ride .proto (inverse Logik: bit=0 = gedrückt)
// https://www.makinolo.com/blog/2024/07/26/zwift-ride-protocol/
struct Btn { uint32_t mask; const char* name; };
static const Btn kButtons[] = {
  { 0x00000001, "LEFT" },   { 0x00000002, "UP" },     { 0x00000004, "RIGHT" },  { 0x00000008, "DOWN" },
  { 0x00000010, "A" },      { 0x00000020, "B" },      { 0x00000040, "Y" },      { 0x00000100, "Z" },  // Z = 0x100 laut .proto
  { 0x00000200, "SHFT_UP_L" }, { 0x00000400, "SHFT_DN_L" },
  { 0x00000800, "POWERUP_L" }, { 0x00001000, "ONOFF_L" },
  { 0x00002000, "SHFT_UP_R" }, { 0x00004000, "SHFT_DN_R" },
  { 0x00010000, "POWERUP_R" }, { 0x00020000, "ONOFF_R" },
};
static const size_t kNumButtons = sizeof(kButtons) / sizeof(kButtons[0]);

static void printPressedButtons(uint32_t buttonMap, const char* prefix = nullptr) {
  auto pressed = [&](uint32_t mask) -> bool { return (buttonMap & mask) == 0; };
  if (prefix) Serial.print(prefix);

#if LOG_COMPACT
  // Eine Zeile: "KEY: LEFT" oder "[L] KEY: LEFT"
  Serial.print("KEY: ");
  bool first = true;
  for (size_t i = 0; i < kNumButtons; i++) {
    if (pressed(kButtons[i].mask)) {
      if (!first) Serial.print(", ");
      Serial.print(kButtons[i].name);
      first = false;
    }
  }
  if (first) Serial.print("(none)");
  Serial.println();
#else
  // Ausführlich: jede Taste eigene Zeile
  for (size_t i = 0; i < kNumButtons; i++) {
    if (pressed(kButtons[i].mask)) Serial.printf("  - %s\n", kButtons[i].name);
  }
#endif
}

// =============================
// NOTIFY / INDICATE callbacks
// =============================
void onMeasNotify(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length == 0) return;

  uint8_t msgId = pData[0];

#if DEBUG_NOTIFY_RAW
  Serial.printf("[NOTIFY] len=%u msgId=0x%02X hex:", (unsigned)length, (unsigned)msgId);
  for (size_t i = 0; i < length && i < (size_t)DEBUG_NOTIFY_MAX_BYTES; i++)
    Serial.printf(" %02X", pData[i]);
  if (length > (size_t)DEBUG_NOTIFY_MAX_BYTES) Serial.print(" ...");
  Serial.println();
#elif LOG_EVT_ALWAYS
  // Kurz bestätigen dass was ankommt (nur bei 0x23 ausführlich, sonst eine Zeile)
  if (msgId != 0x23)
    Serial.printf("EVT msgId=0x%02X len=%u\n", (unsigned)msgId, (unsigned)length);
#endif

  if (msgId == 0x23) lastKeypressTime = millis();

  // Ride keypress message id is 0x23 (Lead + 2. Controller im gleichen ButtonMap)
  if (msgId == 0x23) {
    // After 0x23 follows protobuf RideKeyPadStatus: field 1 = ButtonMap (tag 0x08, varint)
    size_t i = 1;
    uint32_t buttonMap = 0xFFFFFFFF;
    bool parsed = false;

    while (i < length) {
      uint32_t tag = 0;
      if (!readVarint(pData, length, i, tag)) break;
      uint32_t fieldNum = tag >> 3;
      uint32_t wireType = tag & 0x07;

      if (fieldNum == 1 && wireType == 0) { // varint
        uint32_t v = 0;
        if (!readVarint(pData, length, i, v)) break;
        buttonMap = v;
        parsed = true;
        break;
      } else {
        if (wireType == 0) {
          uint32_t tmp;
          if (!readVarint(pData, length, i, tmp)) break;
        } else if (wireType == 2) {
          uint32_t l;
          if (!readVarint(pData, length, i, l)) break;
          i += l;
          if (i > length) break;
        } else {
          break;
        }
      }
    }

    // Fallback: manche Firmwares legen Bitmap direkt nach 0x23 (4 Byte LE)
    if (!parsed && length >= 5) {
      buttonMap = (uint32_t)pData[1] | ((uint32_t)pData[2] << 8) | ((uint32_t)pData[3] << 16) | ((uint32_t)pData[4] << 24);
    }

#if DEBUG_NOTIFY_RAW
    Serial.printf("🎮 Keypress msg 0x23, ButtonMap=0x%08lX\n", (unsigned long)buttonMap);
#endif
    const char* sidePrefix = nullptr;
    if (pDualLeftMeas && pChar == pDualLeftMeas) sidePrefix = "[L] ";
    else if (pDualRightMeas && pChar == pDualRightMeas) sidePrefix = "[R] ";
    printPressedButtons(buttonMap, sidePrefix);
  } else if (DEBUG_NOTIFY_RAW) {
    // Idle 0x19/0x15 – nur bei Debug ausgeben
    Serial.printf("… msgId=0x%02X (len=%u) RAW:", msgId, (unsigned)length);
    for (size_t i = 0; i < length && i < 32; i++) Serial.printf(" %02X", pData[i]);
    if (length > 32) Serial.print(" ...");
    Serial.println();
  }
}

void onRespIndicate(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  // For Ride handshake, controller replies with "RideOn" via indication :contentReference[oaicite:11]{index=11}
  Serial.print("⬅️ INDICATE: ");
  for (size_t i = 0; i < length; i++) Serial.write(pData[i]);
  Serial.println();
}

// =============================
// CLIENT callbacks
// =============================
class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) override {
    Serial.println("✅ Verbunden!");
  }

  void onDisconnect(NimBLEClient* pclient, int reason) override {
    Serial.printf("❌ Getrennt (Code: %d). Rescan...\n", reason);
    pRcSvc = nullptr; pMeas = nullptr; pCtrl = nullptr; pResp = nullptr;
    if (pDeviceCandidate) { delete pDeviceCandidate; pDeviceCandidate = nullptr; }
    scanStartTime = millis();
    NimBLEDevice::getScan()->start(0, false);
  }
};

#if USE_DUAL_CONNECTION
// Disconnect-Callback für Dual: einer weg → beide trennen, Rescan, wieder beide verbinden
class DualClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) override {}
  void onDisconnect(NimBLEClient* pclient, int reason) override {
    Serial.printf("❌ Dual: Controller getrennt (Code: %d). Rescan...\n", reason);
    if (pclient == pClientLeft) pClientLeft = nullptr;
    else if (pclient == pClientRight) pClientRight = nullptr;
    pDualLeftMeas = nullptr; pDualRightMeas = nullptr;
    pDualLeftCtrl = nullptr; pDualRightCtrl = nullptr;
    if (pClientLeft) { pClientLeft->disconnect(); pClientLeft = nullptr; }
    if (pClientRight) { pClientRight->disconnect(); pClientRight = nullptr; }
    if (pDeviceLeft) { delete pDeviceLeft; pDeviceLeft = nullptr; }
    if (pDeviceRight) { delete pDeviceRight; pDeviceRight = nullptr; }
    scanStartTime = millis();
    dualState = 0;  // wieder zuerst Linken (8) suchen
    NimBLEDevice::getScan()->start(0, false);
  }
};
#endif

// =============================
// SCAN callbacks
// =============================
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    std::string name = advertisedDevice->getName();
    if (!name.empty()) {
      Serial.printf("Gefunden: %s %s (RSSI: %d)\n", name.c_str(), advertisedDevice->getAddress().toString().c_str(), advertisedDevice->getRSSI());
    }

    if (name != targetName) return;

    uint8_t devId = 0;  // 0 = unbekannt
    if (advertisedDevice->getManufacturerDataCount() > 0) {
      std::string mfg = advertisedDevice->getManufacturerData(0);
      if (mfg.length() >= 3) {
        uint16_t mfgId = (uint16_t)(uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
        if (mfgId == ZWIFT_MFG_ID) devId = (uint8_t)mfg[2];
      }
    }

#if USE_DUAL_CONNECTION
    // Name-Fallback: „Zwift Click“ auch wenn Werbename gekürzt ist
    bool isZwiftClick = (name == targetName) || (name.find("Zwift Click") != std::string::npos);
    bool asLeft  = (devId == ZWIFT_DEVICE_LEFT) || (devId == 0 && isZwiftClick);
    bool asRight = (devId == ZWIFT_DEVICE_RIGHT) || (devId == 0 && isZwiftClick);
    if (dualState == 0 && asLeft) {
      Serial.printf("📱 Linker (8) gefunden %s – verbinde zuerst mit ihm (Handshake = Signal an rechten).\n", advertisedDevice->getAddress().toString().c_str());
      pDeviceLeft = new NimBLEAdvertisedDevice(*advertisedDevice);
      NimBLEDevice::getScan()->stop();
      doConnectDual = true;
      dualState = 1;
    } else if (dualState == 2 && (asRight || isZwiftClick)) {
      // Rechter: anderes Gerät als Linken (Adresse), sonst gleiches Click zweimal
      if (pDeviceLeft && advertisedDevice->getAddress() == pDeviceLeft->getAddress())
        return;
      Serial.printf("📱 Rechter (7) gefunden %s – verbinde jetzt.\n", advertisedDevice->getAddress().toString().c_str());
      pDeviceRight = new NimBLEAdvertisedDevice(*advertisedDevice);
      NimBLEDevice::getScan()->stop();
      dualState = 3;
    }
    return;
#endif

    // Nur noch rechten (7): 8 hat zuvor keine Events geliefert
    if (onlyRightNextScan) {
      if (devId == ZWIFT_DEVICE_LEFT) return;  // 8 ignorieren
      if (devId == ZWIFT_DEVICE_RIGHT) {
        Serial.printf("🎯 Rechter Controller (7) %s – verbinde (nur 7 nach Timeout).\n", advertisedDevice->getAddress().toString().c_str());
        if (pDeviceCandidate) { delete pDeviceCandidate; pDeviceCandidate = nullptr; }
        pDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
        connectedDeviceId = ZWIFT_DEVICE_RIGHT;
        NimBLEDevice::getScan()->stop();
        doConnect = true;
        return;
      }
    }

    // Normal: Zuerst LINKEN (8) versuchen (Pfeile+YZBA), sonst 7 (YZBA)
    if (devId == ZWIFT_DEVICE_LEFT) {
      Serial.printf("🎯 Linker Controller (8) %s – verbinde (Pfeile+YZBA?).\n", advertisedDevice->getAddress().toString().c_str());
      if (pDeviceCandidate) { delete pDeviceCandidate; pDeviceCandidate = nullptr; }
      pDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
      connectedDeviceId = ZWIFT_DEVICE_LEFT;
      NimBLEDevice::getScan()->stop();
      doConnect = true;
      return;
    }
    if (devId == ZWIFT_DEVICE_RIGHT) {
      Serial.printf("⏳ Rechter Controller (7) %s – warte auf linken (8)...\n", advertisedDevice->getAddress().toString().c_str());
      if (pDeviceCandidate) delete pDeviceCandidate;
      pDeviceCandidate = new NimBLEAdvertisedDevice(*advertisedDevice);
      return;
    }

    // Kein 7/8 → sofort verbinden
    Serial.println("🎯 Verbinde (Device-ID unbekannt)...");
    pDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
    connectedDeviceId = 0;
    NimBLEDevice::getScan()->stop();
    doConnect = true;
  }
};

// =============================
// Find RC service + characteristics
// =============================
static bool pickCharacteristicsByProperties(NimBLERemoteService* svc) {
  // 1) Try known UUIDs (old characteristic UUIDs) first :contentReference[oaicite:12]{index=12}
  pMeas = svc->getCharacteristic(RC_MEAS_CHAR_OLD);
  pCtrl = svc->getCharacteristic(RC_CTRL_CHAR_OLD);
  pResp = svc->getCharacteristic(RC_RESP_CHAR_OLD);

  if (pMeas && pCtrl && pResp) return true;

  // 2) Otherwise enumerate all characteristics and select by properties:
  //    - Measurement: NOTIFY
  //    - Control Point: WRITE or WRITE_NR
  //    - Response: INDICATE (and often READ)
  auto chars = svc->getCharacteristics(true);
  for (auto* c : chars) {
    if (!pMeas && c->canNotify()) pMeas = c;
    if (!pCtrl && (c->canWrite() || c->canWriteNoResponse())) pCtrl = c;
    if (!pResp && c->canIndicate()) pResp = c;
  }

  return (pMeas && pCtrl && pResp);
}

static bool setupRideSession() {
  // Subscribe indications for response characteristic
  if (pResp->canIndicate()) {
    if (!pResp->subscribe(true, onRespIndicate)) {
      Serial.println("⚠️ Konnte Indications nicht abonnieren (Resp).");
    }
  }

  // Ride handshake: write ASCII "RideOn" to Control Point, receive "RideOn" in indication
  const char* rideOn = "RideOn";
  Serial.println("➡️ Sende Handshake: RideOn");
  bool ok = pCtrl->writeValue((uint8_t*)rideOn, strlen(rideOn), true);
  if (!ok) {
    Serial.println("❌ RideOn write fehlgeschlagen.");
    return false;
  }
  delay(400);   // Indication abwarten

  // Optional: Info-Request (Kommando 0, Leer-Nachricht) – manche Firmwares reagieren
  // darauf und melden/koppeln den 2. Controller (Pfeile). Siehe Makinolo Ride-Protokoll.
  const uint8_t infoRequest[] = { 0x00, 0x08, 0x00 };
  Serial.println("➡️ Sende Info-Request (Kopplung 2. Controller?)");
  pCtrl->writeValue((uint8_t*)infoRequest, sizeof(infoRequest), true);
  delay(200);

  // Subscribe notifications for measurement
  Serial.println("📡 Subscribe Measurement notifications...");
  if (!pMeas->subscribe(true, onMeasNotify)) {
    Serial.println("❌ Subscribe auf Measurement fehlgeschlagen.");
    return false;
  }

  connectionTime = millis();
  lastKeypressTime = millis();
  lastKeepaliveTime = millis();
  lastRideOnKeepaliveTime = millis();
  Serial.println("✅ Ready: warte auf Button Events...");
  return true;
}

// Info-Request 00 08 00 als Wake/Keep-Alive an Control Point senden
static void sendKeepalive(NimBLERemoteCharacteristic* ctrl) {
  if (!ctrl) return;
  const uint8_t infoRequest[] = { 0x00, 0x08, 0x00 };
  ctrl->writeValue((uint8_t*)infoRequest, sizeof(infoRequest), true);
}
static void sendRideOnKeepalive(NimBLERemoteCharacteristic* ctrl) {
  if (!ctrl) return;
  const char* rideOn = "RideOn";
  ctrl->writeValue((uint8_t*)rideOn, strlen(rideOn), true);
}

// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Start ESP32 (Zwift Ride / SF2 Reader)...");
  onlyRightNextScan = false;
#if USE_DUAL_CONNECTION
  pDeviceLeft = nullptr; pDeviceRight = nullptr;
  dualState = 0;
  Serial.printf("Suche: '%s' – Dual: zuerst Linken (8), Handshake, dann Rechten (7)\n", targetName.c_str());
#else
  Serial.printf("Suche: '%s' (zuerst linker 8, sonst rechter 7)\n", targetName.c_str());
#endif

  NimBLEDevice::init("ESP32_Zwift_SF2");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new MyScanCallbacks());
  scan->setActiveScan(true);
  scan->setInterval(97);
  scan->setWindow(67);

  scanStartTime = millis();
  scan->start(0, false);
}

// =============================
// LOOP
// =============================
void loop() {
#if USE_DUAL_CONNECTION
  // Phase 0: Timeout ohne Linken (8) → Phase 2: nach Rechten (7) suchen
  if (dualState == 0 && (millis() - scanStartTime >= SCAN_DUAL_WAIT_MS)) {
    Serial.println("⏱️ Kein linker (8) – suche rechten (7)...");
    dualState = 2;  // sofort, damit Callbacks Phase 2 sehen
    NimBLEDevice::getScan()->stop();
    delay(100);  // Scan sauber beenden
    scanStartTime = millis();
    NimBLEDevice::getScan()->start(0, false);
  }
  // Phase 2: Timeout ohne Rechten (7) → nur mit Linkem weiter
  if (dualState == 2 && (millis() - scanStartTime >= SCAN_DUAL_WAIT_MS)) {
    Serial.println("⏱️ Rechter (7) nicht gefunden – nur linker (8) aktiv.");
    NimBLEDevice::getScan()->stop();
    dualState = 4;
  }
#else
  // Nach Timeout: nur rechter (7) gefunden → mit 7 verbinden
  if (pDeviceCandidate && (millis() - scanStartTime >= SCAN_WAIT_MS)) {
    Serial.println("⏱️ Kein linker (8) gefunden – verbinde mit rechten (7).");
    pDevice = pDeviceCandidate;
    pDeviceCandidate = nullptr;
    connectedDeviceId = ZWIFT_DEVICE_RIGHT;
    NimBLEDevice::getScan()->stop();
    doConnect = true;
  }
#endif

#if !USE_DUAL_CONNECTION
  // Mit linker (8) verbunden, aber 8 Sek ohne Tastendruck → trennen, beim nächsten Mal nur 7
  if (pClient && pClient->isConnected() && connectedDeviceId == ZWIFT_DEVICE_LEFT &&
      (millis() - connectionTime >= NO_EVENT_TIMEOUT_MS) &&
      (millis() - lastKeypressTime >= NO_EVENT_TIMEOUT_MS)) {
    Serial.println("⏱️ Linker (8) liefert keine Events – wechsle zu rechten (7).");
    onlyRightNextScan = true;
    connectedDeviceId = 0;
    pClient->disconnect();
  }
#endif

#if USE_DUAL_CONNECTION
  // Phase 1: nur Linken (8) verbinden + Handshake („Signal“), dann Scan für Rechten starten
  if (doConnectDual && dualState == 1 && pDeviceLeft) {
    doConnectDual = false;
    Serial.println("🔗 Verbinde mit linkem (8)...");
    pClient = NimBLEDevice::createClient();
    pClient->setConnectionParams(12, 24, 0, 200);
    pClient->setClientCallbacks(new DualClientCallbacks(), false);
    if (!pClient->connect(pDeviceLeft)) {
      Serial.println("❌ Linker (8) fehlgeschlagen.");
      delete pDeviceLeft; pDeviceLeft = nullptr;
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      dualState = 0; scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    pRcSvc = pClient->getService(RC_SERVICE_NEW);
    if (!pRcSvc) pRcSvc = pClient->getService(RC_SERVICE_OLD);
    if (!pRcSvc || !pickCharacteristicsByProperties(pRcSvc)) {
      Serial.println("❌ Linker (8): Service/Chars fehlgeschlagen.");
      pClient->disconnect();
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      delete pDeviceLeft; pDeviceLeft = nullptr;
      dualState = 0; scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    if (!setupRideSession()) {
      Serial.println("❌ Linker (8): Handshake fehlgeschlagen.");
      pClient->disconnect();
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      delete pDeviceLeft; pDeviceLeft = nullptr;
      dualState = 0; scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    pClientLeft = pClient; pDualLeftMeas = pMeas; pDualLeftCtrl = pCtrl; pClient = nullptr;
    Serial.println("✅ Linker (8) bereit (Handshake = Signal). Suche rechten (7)...");
    dualState = 2;
    scanStartTime = millis();
    NimBLEDevice::getScan()->start(0, false);
    delay(200);
    return;
  }

  // Phase 3: Rechten (7) verbinden (zweiter Client)
  if (dualState == 3 && pDeviceRight) {
    Serial.println("🔗 Verbinde mit rechten (7)...");
    pClient = NimBLEDevice::createClient();
    pClient->setConnectionParams(12, 24, 0, 200);
    pClient->setClientCallbacks(new DualClientCallbacks(), false);
    if (!pClient->connect(pDeviceRight)) {
      Serial.println("❌ Rechter (7) fehlgeschlagen.");
      if (pClientLeft) { pClientLeft->disconnect(); NimBLEDevice::deleteClient(pClientLeft); pClientLeft = nullptr; }
      pDualLeftMeas = nullptr;
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      delete pDeviceLeft; pDeviceLeft = nullptr; delete pDeviceRight; pDeviceRight = nullptr;
      dualState = 0; scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    pRcSvc = pClient->getService(RC_SERVICE_NEW);
    if (!pRcSvc) pRcSvc = pClient->getService(RC_SERVICE_OLD);
    if (!pRcSvc || !pickCharacteristicsByProperties(pRcSvc)) {
      Serial.println("❌ Rechter (7): Service/Chars fehlgeschlagen.");
      pClient->disconnect();
      if (pClientLeft) { pClientLeft->disconnect(); NimBLEDevice::deleteClient(pClientLeft); pClientLeft = nullptr; }
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      pDualLeftMeas = nullptr;
      delete pDeviceLeft; pDeviceLeft = nullptr; delete pDeviceRight; pDeviceRight = nullptr;
      dualState = 0; scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    if (!setupRideSession()) {
      Serial.println("❌ Rechter (7): Handshake fehlgeschlagen.");
      pClient->disconnect();
      if (pClientLeft) { pClientLeft->disconnect(); NimBLEDevice::deleteClient(pClientLeft); pClientLeft = nullptr; }
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      pDualLeftMeas = nullptr;
      delete pDeviceLeft; pDeviceLeft = nullptr; delete pDeviceRight; pDeviceRight = nullptr;
      dualState = 0; scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }
    pClientRight = pClient; pDualRightMeas = pMeas; pDualRightCtrl = pCtrl; pClient = nullptr;
    Serial.println("✅ Beide verbunden. [L]=Pfeile, [R]=YZBA+");
    dualState = 4;
    delay(200);
    return;
  }
#endif

  // Keep-Alive: Info-Request alle 5 s + optional RideOn alle 30 s (Clicks schlafen sonst nach ~1 min ein)
  if (KEEPALIVE_INTERVAL_MS > 0 || RIDEON_KEEPALIVE_MS > 0) {
    uint32_t now = millis();
#if USE_DUAL_CONNECTION
    if (dualState == 4) {
      if (KEEPALIVE_INTERVAL_MS > 0 && (now - lastKeepaliveTime >= KEEPALIVE_INTERVAL_MS)) {
        lastKeepaliveTime = now;
        if (pDualLeftCtrl)  sendKeepalive(pDualLeftCtrl);
        if (pDualRightCtrl) sendKeepalive(pDualRightCtrl);
      }
      if (RIDEON_KEEPALIVE_MS > 0 && (now - lastRideOnKeepaliveTime >= RIDEON_KEEPALIVE_MS)) {
        lastRideOnKeepaliveTime = now;
        if (pDualLeftCtrl)  sendRideOnKeepalive(pDualLeftCtrl);
        if (pDualRightCtrl) sendRideOnKeepalive(pDualRightCtrl);
      }
    }
#else
    if (pClient && pClient->isConnected() && pCtrl) {
      if (KEEPALIVE_INTERVAL_MS > 0 && (now - lastKeepaliveTime >= KEEPALIVE_INTERVAL_MS)) {
        lastKeepaliveTime = now;
        sendKeepalive(pCtrl);
      }
      if (RIDEON_KEEPALIVE_MS > 0 && (now - lastRideOnKeepaliveTime >= RIDEON_KEEPALIVE_MS)) {
        lastRideOnKeepaliveTime = now;
        sendRideOnKeepalive(pCtrl);
      }
    }
#endif
  }

  if (doConnect) {
    doConnect = false;

    Serial.println("🔗 Verbinde...");

    if (pClient == nullptr) {
      pClient = NimBLEDevice::createClient();
      pClient->setConnectionParams(12, 24, 0, 200);
      pClient->setClientCallbacks(new MyClientCallback(), false);
    }

    if (!pClient->connect(pDevice)) {
      Serial.println("❌ Verbindung fehlgeschlagen.");
      NimBLEDevice::deleteClient(pClient); pClient = nullptr;
      if (pDeviceCandidate) { delete pDeviceCandidate; pDeviceCandidate = nullptr; }
      scanStartTime = millis();
      NimBLEDevice::getScan()->start(0, false);
      delay(200);
      return;
    }

    // Get RC service: prefer NEW FC82, fallback OLD 128-bit :contentReference[oaicite:14]{index=14}
    pRcSvc = pClient->getService(RC_SERVICE_NEW);
    if (!pRcSvc) pRcSvc = pClient->getService(RC_SERVICE_OLD);

    if (!pRcSvc) {
      Serial.println("❌ RC Service nicht gefunden (weder FC82 noch 00000001...).");
      pClient->disconnect();
      return;
    }
    Serial.println("✅ RC Service gefunden.");

    if (!pickCharacteristicsByProperties(pRcSvc)) {
      Serial.println("❌ Konnte RC Characteristics nicht sauber bestimmen.");
      pClient->disconnect();
      return;
    }

    Serial.printf("✅ MEAS UUID: %s\n", pMeas->getUUID().toString().c_str());
    Serial.printf("✅ CTRL UUID: %s\n", pCtrl->getUUID().toString().c_str());
    Serial.printf("✅ RESP UUID: %s\n", pResp->getUUID().toString().c_str());

    if (!setupRideSession()) {
      Serial.println("❌ Setup Session fehlgeschlagen, disconnect...");
      pClient->disconnect();
      return;
    }
  }

  delay(200);
}
