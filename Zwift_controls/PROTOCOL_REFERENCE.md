# Zwift Ride / Click – BLE-Protokoll (Referenz)

Kurzreferenz basierend auf [Makinolo: Zwift Ride protocol](https://www.makinolo.com/blog/2024/07/26/zwift-ride-protocol/) und Januar-2025-Update.

## Geräte

| Device ID (Mfg-Daten 0x094A) | Gerät        | Rolle |
|------------------------------|-------------|--------|
| 7 | Ride Right | Rechter Controller (YZBA+) |
| 8 | Ride Left  | Linker Controller (Pfeile); **Aggregator** – alle Tasten durch einen BLE-Link |
| 9 | Click      | Zwift Click |

- **Advertised name:** „Zwift SF2“ (Ride) bzw. „Zwift Click“ (Click).
- **Empfehlung im Artikel:** Nur mit dem **linken (8)** verbinden, dann kommen beide Controller durch. In der Praxis kann bei Click/Ride je nach Setup nur der rechte (7) zuverlässig Events liefern → Sketch unterstützt beide Strategien.

## Service-UUIDs

- **Alt (bis ca. Jan 2025):** `00000001-19ca-4651-86e5-fa29dcdd09d1`
- **Neu (FC82):** 16-Bit-UUID `0xFC82`

Der Sketch fragt zuerst FC82 ab, sonst die alte UUID.

## Handshake

1. An **Write-Characteristic** ASCII-String **`RideOn`** schreiben.
2. Antwort in **Indication-Characteristic**: ebenfalls **`RideOn`**.
3. Optional: Info-Request `00 08 00` an Write-Char (Antwort in Indication).

## Keypress (Notification)

- **Message-ID:** erstes Byte **`0x23`**.
- Danach Protobuf **RideKeyPadStatus**: Feld 1 = **ButtonMap** (32-Bit, Varint), Feld 2 = Analog (Joystick).
- **Logik:** Bit = 0 → Taste gedrückt; Bit = 1 → nicht gedrückt.

### RideButtonMask (.proto)

| Maske   | Name        |
|---------|-------------|
| 1       | LEFT        |
| 2       | UP          |
| 4       | RIGHT       |
| 8       | DOWN        |
| 0x10    | A           |
| 0x20    | B           |
| 0x40    | Y           |
| **0x100** | **Z**    |
| 0x200   | SHFT_UP_L   |
| 0x400   | SHFT_DN_L   |
| 0x800   | POWERUP_L   |
| 0x1000  | ONOFF_L     |
| 0x2000  | SHFT_UP_R   |
| 0x4000  | SHFT_DN_R   |
| 0x10000 | POWERUP_R   |
| 0x20000 | ONOFF_R     |

Idle-Messages: `0x19` oder `0x15` (periodisch, keine Tasten).

## Control-Point-Befehle (Write)

- **Info-Request:** `00 08 00` (Kommando 0, leeres Varint-Feld). Antwort in Indication (u. a. Name „Zwift SF2“, Version).
- **Parameter 770:** `00 08 82 06` → Antwort enthält u. a. „sf2“, „nrf52“ (undekodiert).

---

*Quelle: Makinolo Blog (Zwift Ride protocol), Edit Januar 2025 (Service UUID FC82).*
