#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <cstring>

// --- Konstanty protokolu ---

const uint8_t ST_START_BYTE      = 0x7E;
const uint8_t ST_STOP_BYTE       = 0x81;
const uint8_t ST_MAX_PACKET_SIZE = 0xFE;  // 254 bajtu payload

// Velikosti preamble/postamble
const uint8_t ST_PREAMBLE_SIZE_COMPAT   = 4;  // START, ID, COBS_OVERHEAD, LEN
const uint8_t ST_PREAMBLE_SIZE_ENHANCED = 5;  // START, ID, SEQ, COBS_OVERHEAD, LEN
const uint8_t ST_POSTAMBLE_SIZE_COMPAT  = 2;  // CRC8, STOP
const uint8_t ST_POSTAMBLE_SIZE_ENHANCED = 3; // CRC16_H, CRC16_L, STOP

// --- Stavove kody ---

const int8_t ST_CONTINUE           =  3;
const int8_t ST_NEW_DATA           =  2;
const int8_t ST_NO_DATA            =  1;
const int8_t ST_CRC_ERROR          =  0;
const int8_t ST_PAYLOAD_ERROR      = -1;
const int8_t ST_STOP_BYTE_ERROR    = -2;
const int8_t ST_STALE_PACKET_ERROR = -3;

// --- Typ pro callbacky ---

typedef void (*stCallbackPtr)();


class AP_SerialTransfer
{
public:
    /**
     * @brief Konstruktor
     * @param tag Nazev pro logovani (default "AP_SerialTransfer")
     * @param arduinoCompatible true = kompatibilni rezim s Arduino SerialTransfer
     *                          false = vylepseny rezim (CRC-16, sekvencni cisla)
     */
    AP_SerialTransfer(const char *tag = "AP_SerialTransfer", bool arduinoCompatible = false);

    /**
     * @brief Inicializace UART a paketoveho protokolu
     * @param uartNum Cislo UART portu (UART_NUM_0, UART_NUM_1, UART_NUM_2)
     * @param txPin GPIO pin pro TX
     * @param rxPin GPIO pin pro RX
     * @param baudRate Prenosova rychlost (default 115200)
     * @param rxBufSize Velikost RX ring bufferu (default 1024)
     */
    void begin(uart_port_t uartNum, int txPin, int rxPin,
               int baudRate = 115200, size_t rxBufSize = 1024);

    /**
     * @brief Odesle data z txBuff jako paket
     * @param messageLen Pocet bajtu v txBuff k odeslani
     * @param packetID Identifikator paketu (default 0)
     * @return Pocet odeslanych payload bajtu
     */
    uint8_t sendData(uint16_t messageLen, uint8_t packetID = 0);

    /**
     * @brief Kontrola a parsovani prijatych dat (non-blocking)
     * @return Pocet prijatych payload bajtu (0 = zadna data)
     */
    uint8_t available();

    /**
     * @brief Wrapper pro available(), vhodny pro pouziti s callbacky
     * @return true pokud byl prijat kompletni paket
     */
    bool tick();

    /**
     * @brief Vrati ID posledniho prijateho paketu
     * @return Packet ID (0-255)
     */
    uint8_t currentPacketID();

    /**
     * @brief Vrati sekvencni cislo posledniho prijateho paketu
     *        V kompatibilnim rezimu vraci vzdy 0
     * @return Sequence number (0-255)
     */
    uint8_t currentSeqNum();

    /**
     * @brief Reset bufferu a stavoveho automatu, vyprazdni UART RX buffer
     */
    void reset();

    /**
     * @brief Nastavi pole callback funkci
     *        Callback na indexu N se zavola pri prijmu paketu s ID = N
     * @param callbacks Pole ukazatelu na funkce
     * @param count Pocet callbacku v poli
     */
    void setCallbacks(const stCallbackPtr *callbacks, uint8_t count);

    /**
     * @brief Nastavi timeout pro parsovani paketu
     * @param timeoutMs Timeout v milisekundach (default 50)
     */
    void setTimeout(uint32_t timeoutMs);

    // --- Template metody pro praci s daty ---

    /**
     * @brief Vlozi libovolny objekt do TX bufferu
     * @param val Objekt k odeslani (struct, int, float, ...)
     * @param index Pocatecni pozice v txBuff (default 0)
     * @param len Pocet bajtu k zapisu (default sizeof(T))
     * @return Index nasledujici za zapsanymi daty
     */
    template <typename T>
    uint16_t txObj(const T &val, uint16_t index = 0, uint16_t len = sizeof(T))
    {
        const uint8_t *ptr = (const uint8_t *)&val;
        uint16_t maxIndex;

        if ((len + index) > ST_MAX_PACKET_SIZE)
            maxIndex = ST_MAX_PACKET_SIZE;
        else
            maxIndex = len + index;

        for (uint16_t i = index; i < maxIndex; i++)
        {
            txBuff[i] = *ptr;
            ptr++;
        }

        return maxIndex;
    }

    /**
     * @brief Precte libovolny objekt z RX bufferu
     * @param val Objekt kam se data nakopiruji
     * @param index Pocatecni pozice v rxBuff (default 0)
     * @param len Pocet bajtu k precteni (default sizeof(T))
     * @return Index nasledujici za prectenymi daty
     */
    template <typename T>
    uint16_t rxObj(const T &val, uint16_t index = 0, uint16_t len = sizeof(T))
    {
        uint8_t *ptr = (uint8_t *)&val;
        uint16_t maxIndex;

        if ((len + index) > ST_MAX_PACKET_SIZE)
            maxIndex = ST_MAX_PACKET_SIZE;
        else
            maxIndex = len + index;

        for (uint16_t i = index; i < maxIndex; i++)
        {
            *ptr = rxBuff[i];
            ptr++;
        }

        return maxIndex;
    }

    /**
     * @brief Odesle jeden objekt jako samostatny paket (zkratka)
     * @param val Objekt k odeslani
     * @param len Pocet bajtu (default sizeof(T))
     * @return Pocet odeslanych payload bajtu
     */
    template <typename T>
    uint8_t sendDatum(const T &val, uint16_t len = sizeof(T))
    {
        return sendData(txObj(val, 0, len));
    }

    // --- Verejne buffery a stav ---

    uint8_t txBuff[ST_MAX_PACKET_SIZE];
    uint8_t rxBuff[ST_MAX_PACKET_SIZE];
    uint8_t bytesRead;
    int8_t  status;

private:
    const char *_tag;
    uart_port_t _uartNum;
    bool _initialized;
    bool _arduinoCompat;

    // --- Paketovy protokol ---

    uint8_t _preamble[ST_PREAMBLE_SIZE_ENHANCED];   // Max velikost (5 bajtu)
    uint8_t _postamble[ST_POSTAMBLE_SIZE_ENHANCED];  // Max velikost (3 bajty)
    uint8_t _preambleSize;
    uint8_t _postambleSize;
    uint8_t _overheadByte;
    uint8_t _recOverheadByte;

    // --- Sekvencni cislo (enhanced mode) ---

    uint8_t _txSeqNum;   // Citac pro odesilani (auto-increment)
    uint8_t _rxSeqNum;   // Posledni prijate sekvencni cislo

    // --- Stavovy automat parseru ---

    enum _fsm
    {
        find_start_byte,
        find_id_byte,
        find_seq_byte,       // Pouze enhanced mode
        find_overhead_byte,
        find_payload_len,
        find_payload,
        find_crc,            // Compat: CRC-8, Enhanced: CRC-16 high byte
        find_crc_low,        // Pouze enhanced mode: CRC-16 low byte
        find_end_byte
    };
    _fsm _state;

    uint8_t  _idByte;
    uint8_t  _bytesToRec;
    uint8_t  _payIndex;
    uint32_t _packetStart;
    uint32_t _timeout;
    uint8_t  _recCrcHigh;  // Enhanced mode: ulozeny high byte CRC-16

    // --- Callbacky ---

    const stCallbackPtr *_callbacks;
    uint8_t _callbacksLen;

    // --- CRC-8 (kompatibilni rezim, polynom 0x9B) ---

    uint8_t _crcTable8[256];
    void    _generateCrcTable8(uint8_t polynomial = 0x9B);
    uint8_t _calcCrc8(uint8_t *data, uint8_t len);

    // --- CRC-16 (enhanced rezim, CRC-16/CCITT-FALSE, polynom 0x1021) ---

    uint16_t _crcTable16[256];
    void     _generateCrcTable16(uint16_t polynomial = 0x1021);
    uint16_t _calcCrc16(uint8_t *data, uint8_t len);

    // --- COBS kodovani ---

    void    _calcOverhead(uint8_t *data, uint8_t len);
    int16_t _findLast(uint8_t *data, uint8_t len);
    void    _stuffPacket(uint8_t *data, uint8_t len);
    void    _unpackPacket(uint8_t *data);

    // --- Paketove operace ---

    uint8_t _constructPacket(uint16_t messageLen, uint8_t packetID);
    uint8_t _parse(uint8_t recChar, bool valid);

    // --- Mutex pro thread-safe odesilani ---

    SemaphoreHandle_t _txMutex;
};
