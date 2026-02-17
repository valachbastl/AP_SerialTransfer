# Changelog

## [1.0.0] - 2026-02-09

### Added
- Dva rezimy provozu: **enhanced** (default) a **arduino-compat**
- Paketova komunikace pres UART s ESP-IDF driverem
- COBS kodovani (Consistent Overhead Byte Stuffing) pro spolehlivy prenos

#### Enhanced rezim (default)
- CRC-16/CCITT-FALSE (polynom 0x1021) pro silnou detekci chyb
- Sekvencni cisla paketu (0-255, auto-increment) pro detekci ztracenych/zduplikovanych paketu
- Format: `START | ID | SEQ | COBS | LEN | payload | CRC16_H | CRC16_L | STOP`
- Metoda `currentSeqNum()` pro cteni sekvencniho cisla prijateho paketu

#### Arduino-compat rezim
- CRC-8 (polynom 0x9B) — plna kompatibilita s Arduino SerialTransfer
- Format: `START | ID | COBS | LEN | payload | CRC8 | STOP`
- Zapnuti: `AP_SerialTransfer transfer("tag", true)`

#### Spolecne
- Template metody `txObj()`, `rxObj()` pro odesilani/prijem libovolnych datovych typu
- Zkratka `sendDatum()` pro odeslani jednoho objektu jako paketu
- Non-blocking prijem dat metodou `available()`
- Podpora callback funkci mapovanych na packet ID (`setCallbacks()`, `tick()`)
- Thread-safe odesilani (interni mutex) - bezpecne volani `sendData()` z vice RTOS tasku
- Konfigurovatelny timeout pro detekci neuplnych paketu (`setTimeout()`)
- ESP-IDF logovani chyb a stavu (ESP_LOGI, ESP_LOGW, ESP_LOGE)
