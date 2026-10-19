# Changelog

## [1.2.0] - 2026-10-19

### Added
- `rxPacket(val)` — prisne precteni celeho paketu: overi, ze prijaty payload ma presne `sizeof(T)` bajtu, jinak vrati `false` a objekt nemeni. `rxObj()` delku nekontroluje — kratsi paket (napr. jina verze struktury na druhe strane) doplnil zbytek objektu starymi bajty z `rxBuff`.
- `rxPacket(val, expectedID)` — navic overi ID paketu (typ/verze zpravy).
- `sendPacket(val, packetID)` — protejsek `rxPacket()`: odesle cely objekt s danym ID (`sendDatum()` ID nastavit neumi).
- `static_assert` na velikost objektu u obou novych metod (objekt > 254 B = chyba prekladu).

### Fixed
- **Prijem prestal fungovat po 49,7 dnech behu:** `_packetStart` byl `uint32_t`, ale porovnaval se s 64bitovym casem v ms (`esp_timer_get_time() / 1000`). Po 2^32 ms vychazel rozdil vzdy ~2^32 → kazdy paket skoncil jako `STALE_PACKET_ERROR` az do restartu. `_packetStart` je nyni `uint64_t`.
- Chyba ramce (CRC, stop byte, delka, timeout) uz nevyprazdnuje cely UART RX buffer — drive jeden poskozeny bajt zahodil i vsechny platne pakety, ktere za nim uz cekaly. Parser se jen resetuje a hleda dalsi start byte (falesny start odmitne delka/CRC/stop byte). Rucni `reset()` dal buffer vyprazdnuje.
- Bajt, na kterem se odhali timeout nebo chybejici stop byte, se uz nezahazuje — pokud je to start byte, zacne se jim rovnou novy paket (useknuty ramec nasledovany novym neztrati i ten novy).
- `sendDatum()` nebyl thread-safe, prestoze knihovna slibuje bezpecne odesilani z vice tasku: objekt se kopiroval do sdileneho `txBuff` pres `txObj()` mimo TX mutex, takze dva soubezne odesilajici tasky si mohly data prepsat. Kopie i odeslani ted probehnou pod jednim drzenim mutexu (stejne `sendPacket()`). Rucni sekvence `txObj()` + `sendData()` zustava ne-atomicka — zdokumentovano v hlavicce.
- `sendDatum(val, len)` s `len > sizeof(T)` cetl za koncem objektu — `len` se nyni orezava na `sizeof(T)`.
- `begin()` je idempotentni (druhe volani po uspechu vrati `ESP_OK`, driv selhalo na `uart_driver_install`) a pri selhani `uart_param_config`/`uart_set_pin` odinstaluje UART driver, takze jde `begin()` zopakovat.

### Changed
- TX mutex staticky alokovany (`StaticSemaphore_t` + `xSemaphoreCreateMutexStatic()`) v konstruktoru misto `xSemaphoreCreateMutex()` v `begin()` — nemuze selhat na OOM, odpada `ESP_ERR_NO_MEM` vetev v `begin()`. Vyzaduje `CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION=y` (ESP-IDF default).
- Trida je nekopirovatelna (copy ctor/assign `= delete`) — kopie by rozbila staticky mutex. Kopirovani objektu nikdy nedavalo smysl (vlastni UART driver).
- Komentare a log texty v kodu prepsany do anglictiny.
- `TODO_next_version.md` pridan do `.gitignore` (lokalni pracovni poznamka).
- Jinak `rxObj()`, `sendDatum()` a ostatni API beze zmeny (zpetne kompatibilni).

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
