# Changelog

## [1.1.1] - 2026-06-16

### Added
- ESP-IDF component baleni (`CMakeLists.txt` + `idf_component.yml`) - pouzitelne jako cista ESP-IDF komponenta, nejen pres PlatformIO. Bez zmeny kodu.

## [1.1.0] - 2026-06-11

### Fixed
- `_unpackPacket()` ohranicen delkou payloadu (`_bytesToRec`) — opraven off-by-one guard a neohranicena COBS smycka, ktere umoznovaly zapis mimo `rxBuff` pri poskozenych / CRC-kolidujicich datech z drátu.
- `reset()` uz nemaze `txBuff` — mazal TX buffer z RX cesty (semanticky spatne + race se `sendData()` pres `_txMutex`). Nyni resetuje jen RX stav (parser + `rxBuff` + UART flush).
- `rxObj()` ma parametr `T&` misto `const T&` — zapis do objektu uz neni pres const referenci (const-correctness; shodne s Arduino SerialTransfer API). Volajici predavaji non-const objekty, takze stare API funguje dal.

### Added
- `begin()` nyni vraci `esp_err_t` (drive `void`) — volajici muze osetrit selhani UART initu. Stare API zustava funkcni (navratovku lze ignorovat).
- `isInitialized()` — dotaz na uspesnost `begin()`.
- `begin()` kontroluje alokaci TX mutexu (OOM → `ESP_ERR_NO_MEM`).

### Changed
- Licence zmenena na MIT (drive UNLICENSED).

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
