# Zwift Play – BLE-Protokoll (Referenz)

**Hinweis:** Zwift **Play** ist die ältere Gamepad-Serie (2 getrennte Controller für Lenker links/rechts), **verschlüsselt**.  
Zwift **Ride** / **Click** (SF2) nutzen ein anderes, unverschlüsseltes Protokoll – siehe `PROTOCOL_REFERENCE.md`.

Quelle: Makinolo (makinolo at gmail). Reverse Engineering, nicht offiziell von Zwift.

---

## Geräte

| Device ID (Mfg 0x094A) | Gerät        |
|------------------------|-------------|
| 2 | Play Right |
| 3 | Play Left  |
| 9 | Click      |

- **Advertised name:** „Zwift Play“ (beide Controller).
- **Services:** DIS (180A), BAS (180F), **Zwift RC Service** `00000001-19ca-4651-86e5-fa29dcdd09d1`.

---

## Zwift Play Service (RC1)

| Charakteristik | UUID | Rolle |
|----------------|------|--------|
| Measurement    | 00000002-19ca-4651-86e5-fa29dcdd09d1 | Notify |
| Control Point  | 00000003-19ca-4651-86e5-fa29dcdd09d1 | Write |
| Response      | 00000004-19ca-4651-86e5-fa29dcdd09d1 | Indicate, Read |

- Ohne **Handshake** (Schlüsseltausch) liefert Measurement **keine** Daten.
- **Verschlüsselung:** ECDH (P-256) → HKDF (SHA256, 36 Byte Key) → AES-256-CCM für alle Nachrichten.

---

## Handshake (Schlüsseltausch)

1. **App → Gerät:** An Control Point schreiben:  
   `52 69 64 65 4F 6E 01 02` + **App-Public-Key** (unkomprimiert, ohne führendes 0x04).
2. **Gerät → App:** Indication:  
   `52 69 64 65 4F 6E 00 09` (oder `01 03`) + **Gerät-Public-Key**.
3. **Shared Secret:** ECDH(App-Private, Gerät-Public).
4. **Symmetric Key:** HKDF(Secret, SHA256, 36 Byte, Salt = Gerät-Public || App-Public).

Danach: alle weiteren Nachrichten (Notify/Indication/Write) mit AES-CCM (8-Byte-Nonce: letzte 4 Byte des Keys + 4-Byte-Counter, 4-Byte-MIC).

---

## Nachrichten (nach Entschlüsselung)

- **Idle:** 1 Byte `0x15` (~1 Hz).
- **Tastendruck:** Beginnt mit Opcode **0x07**, danach Protobuf-ähnliche Tag-Value-Paare (Varint).

### Button-Tags (Beispiel)

| TAG (hex) | Field | Bedeutung |
|-----------|--------|-----------|
| 0x08 | 1 | Pad: 0 = Right, 1 = Left |
| 0x10 | 2 | Y / ^ |
| 0x18 | 3 | Z / < |
| 0x20 | 4 | A / > |
| 0x28 | 5 | B / v |
| 0x30 | 6 | ON/OFF |
| 0x38 | 7 | Shifter |
| 0x40 | 8 | Orange Joystick Left/Right (Varint, LSB = Richtung) |
| 0x48 | 9 | Orange Joystick Brake (Wert 0–200) |

Wert 0 = gedrückt, 1 = nicht gedrückt (bei normalen Tasten). Joystick: signed Varint (Zickzack), Bereich z. B. 0–100 für Lenkung.

---

## Control-Point-Befehle (verschlüsselt)

| Cmd | Parameter (Beispiel) | Wirkung |
|-----|----------------------|--------|
| 0x12 | 0x12,0x08,0x0A,0x06,0x08,0x02,0x10,0x00,0x18,[pattern] | Haptic (Vibration), 123 Muster |
| 0x18 | 0x05 | Reset |

Weitere Befehle (0x00, 0x02, 0x08, 0x0A, 0x28, 0x2D, 0x38, 0x48) in der Doku erwähnt, Bedeutung unklar.

---

## Unterschied zu Ride / Click

| | Zwift **Play** | Zwift **Ride** / **Click** |
|---|----------------|----------------------------|
| Verschlüsselung | Ja (ECDH + AES-CCM) | Nein |
| Handshake | Schlüsseltausch | ASCII „RideOn“ + ggf. Info-Request |
| Keypress | Opcode 0x07, Tag-Value-Liste | Message-ID 0x23, 32-Bit-ButtonMap (invers) |
| Device IDs | 2 = Right, 3 = Left | 7 = Ride Right, 8 = Ride Left, 9 = Click |
| Service | 00000001-19ca-... (Ride auch FC82) | Wie Play oder FC82 |

Der Sketch in diesem Ordner (`Zwift_controls.ino`) ist für **Ride/Click** (unverschlüsselt, 0x23-Keypad). Für **Zwift Play** wäre ein eigener Client mit Crypto (ECDH, HKDF, AES-CCM) nötig.
