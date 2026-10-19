#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <cstring>

// --- Protocol constants ---

const uint8_t ST_START_BYTE      = 0x7E;
const uint8_t ST_STOP_BYTE       = 0x81;
const uint8_t ST_MAX_PACKET_SIZE = 0xFE;  // 254 bytes of payload

// Preamble/postamble sizes
const uint8_t ST_PREAMBLE_SIZE_COMPAT   = 4;  // START, ID, COBS_OVERHEAD, LEN
const uint8_t ST_PREAMBLE_SIZE_ENHANCED = 5;  // START, ID, SEQ, COBS_OVERHEAD, LEN
const uint8_t ST_POSTAMBLE_SIZE_COMPAT  = 2;  // CRC8, STOP
const uint8_t ST_POSTAMBLE_SIZE_ENHANCED = 3; // CRC16_H, CRC16_L, STOP

// --- Status codes ---

const int8_t ST_CONTINUE           =  3;
const int8_t ST_NEW_DATA           =  2;
const int8_t ST_NO_DATA            =  1;
const int8_t ST_CRC_ERROR          =  0;
const int8_t ST_PAYLOAD_ERROR      = -1;
const int8_t ST_STOP_BYTE_ERROR    = -2;
const int8_t ST_STALE_PACKET_ERROR = -3;

// --- Callback type ---

typedef void (*stCallbackPtr)();


/**
 * Packet-based UART transport (COBS framing, CRC-8 or CRC-16).
 *
 * Thread safety:
 *  - TX: sendData(), sendDatum() and sendPacket() are safe to call from multiple
 *    tasks. sendDatum()/sendPacket() copy the object into txBuff under the TX
 *    mutex, so the whole send is atomic. A manual txObj() + sendData() sequence
 *    is NOT atomic across tasks (txBuff is shared) — use it from one task only.
 *  - RX: available()/tick()/rxObj()/rxPacket() must be called from a single task.
 */
class AP_SerialTransfer
{
public:
    /**
     * @brief Constructor — only stores parameters, does not touch the UART
     * @param tag Log tag (default "AP_SerialTransfer")
     * @param arduinoCompatible true  = compatible with Arduino SerialTransfer (CRC-8)
     *                          false = enhanced mode (CRC-16, sequence numbers)
     */
    AP_SerialTransfer(const char *tag = "AP_SerialTransfer", bool arduinoCompatible = false);

    // Owns a UART driver and a statically allocated mutex — not copyable.
    AP_SerialTransfer(const AP_SerialTransfer &) = delete;
    AP_SerialTransfer &operator=(const AP_SerialTransfer &) = delete;

    /**
     * @brief Install and configure the UART driver
     *        Idempotent — a second call after success returns ESP_OK without
     *        touching the UART. On failure the driver is uninstalled again, so
     *        begin() can simply be retried.
     * @param uartNum UART port (UART_NUM_0, UART_NUM_1, UART_NUM_2)
     * @param txPin TX GPIO
     * @param rxPin RX GPIO
     * @param baudRate Baud rate (default 115200)
     * @param rxBufSize RX ring buffer size (default 1024)
     * @return ESP_OK on success, otherwise the error from the UART driver.
     *         (The return value may be ignored — the old API still works.)
     */
    esp_err_t begin(uart_port_t uartNum, int txPin, int rxPin,
                    int baudRate = 115200, size_t rxBufSize = 1024);

    /**
     * @brief True if begin() succeeded
     */
    bool isInitialized() const { return _initialized; }

    /**
     * @brief Send the contents of txBuff as one packet
     * @param messageLen Number of bytes in txBuff to send
     * @param packetID Packet identifier (default 0)
     * @return Number of payload bytes sent
     */
    uint8_t sendData(uint16_t messageLen, uint8_t packetID = 0);

    /**
     * @brief Poll and parse received data (non-blocking)
     * @return Number of payload bytes received (0 = no complete packet)
     */
    uint8_t available();

    /**
     * @brief Wrapper around available(), intended for use with callbacks
     * @return true if a complete packet was received
     */
    bool tick();

    /**
     * @brief ID of the last received packet
     * @return Packet ID (0-255)
     */
    uint8_t currentPacketID();

    /**
     * @brief Sequence number of the last received packet
     *        Always 0 in compatible mode
     * @return Sequence number (0-255)
     */
    uint8_t currentSeqNum();

    /**
     * @brief Reset the RX buffer and parser state, flush the UART RX buffer
     *        Manual use only — framing errors (CRC, stop byte, timeout) resync
     *        on the next start byte without flushing, so valid packets already
     *        buffered behind a corrupted one are not lost.
     */
    void reset();

    /**
     * @brief Set the callback array
     *        The callback at index N is called when a packet with ID = N arrives
     * @param callbacks Array of function pointers
     * @param count Number of callbacks in the array
     */
    void setCallbacks(const stCallbackPtr *callbacks, uint8_t count);

    /**
     * @brief Set the packet parse timeout
     * @param timeoutMs Timeout in milliseconds (default 50)
     */
    void setTimeout(uint32_t timeoutMs);

    // --- Template data methods ---

    /**
     * @brief Write any object into txBuff
     *        Not atomic together with sendData() — see thread safety note above.
     * @param val Object to send (struct, int, float, ...)
     * @param index Start position in txBuff (default 0)
     * @param len Number of bytes to write (default sizeof(T))
     * @return Index just past the written data
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
     * @brief Read any object from rxBuff (no length check, see rxPacket())
     * @param val Destination object
     * @param index Start position in rxBuff (default 0)
     * @param len Number of bytes to read (default sizeof(T))
     * @return Index just past the read data
     */
    template <typename T>
    uint16_t rxObj(T &val, uint16_t index = 0, uint16_t len = sizeof(T))
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
     * @brief Send one object as a single packet with ID 0 (shortcut)
     * @param val Object to send
     * @param len Number of bytes (default sizeof(T), clipped to ST_MAX_PACKET_SIZE)
     * @return Number of payload bytes sent
     */
    template <typename T>
    uint8_t sendDatum(const T &val, uint16_t len = sizeof(T))
    {
        if (len > sizeof(T)) len = sizeof(T);
        return _sendBytes(&val, len, 0);
    }

    /**
     * @brief Send a whole object as a single packet with the given ID
     *        Counterpart of rxPacket() — the ID typically carries the message
     *        type / protocol version.
     * @param val Object to send (all sizeof(T) bytes)
     * @param packetID Packet identifier (default 0)
     * @return Number of payload bytes sent
     */
    template <typename T>
    uint8_t sendPacket(const T &val, uint8_t packetID = 0)
    {
        static_assert(sizeof(T) <= ST_MAX_PACKET_SIZE, "Object does not fit into one packet");
        return _sendBytes(&val, sizeof(T), packetID);
    }

    /**
     * @brief Strict read of a whole packet into an object
     *        Call right after available() > 0. Unlike rxObj(), checks that the
     *        received payload is exactly sizeof(T) bytes — a shorter/longer
     *        packet (e.g. a different struct version on the other side) is
     *        rejected and val is left untouched, instead of being filled with
     *        stale bytes from rxBuff.
     * @param val Destination object
     * @return true if the length matches and the data was copied
     */
    template <typename T>
    bool rxPacket(T &val)
    {
        static_assert(sizeof(T) <= ST_MAX_PACKET_SIZE, "Object does not fit into one packet");
        if (bytesRead != sizeof(T))
            return false;
        memcpy(&val, rxBuff, sizeof(T));
        return true;
    }

    /**
     * @brief Strict read of a packet, checking both length and ID
     * @param val Destination object
     * @param expectedID Expected packet ID (see sendPacket())
     * @return true if both length and ID match and the data was copied
     */
    template <typename T>
    bool rxPacket(T &val, uint8_t expectedID)
    {
        if (_idByte != expectedID)
            return false;
        return rxPacket(val);
    }

    // --- Public buffers and state ---

    uint8_t txBuff[ST_MAX_PACKET_SIZE];
    uint8_t rxBuff[ST_MAX_PACKET_SIZE];
    uint8_t bytesRead;
    int8_t  status;

private:
    const char *_tag;
    uart_port_t _uartNum;
    bool _initialized;
    bool _arduinoCompat;

    // --- Packet protocol ---

    uint8_t _preamble[ST_PREAMBLE_SIZE_ENHANCED];   // Max size (5 bytes)
    uint8_t _postamble[ST_POSTAMBLE_SIZE_ENHANCED];  // Max size (3 bytes)
    uint8_t _preambleSize;
    uint8_t _postambleSize;
    uint8_t _overheadByte;
    uint8_t _recOverheadByte;

    // --- Sequence numbers (enhanced mode) ---

    uint8_t _txSeqNum;   // TX counter (auto-increment)
    uint8_t _rxSeqNum;   // Last received sequence number

    // --- Parser state machine ---

    enum _fsm
    {
        find_start_byte,
        find_id_byte,
        find_seq_byte,       // Enhanced mode only
        find_overhead_byte,
        find_payload_len,
        find_payload,
        find_crc,            // Compat: CRC-8, Enhanced: CRC-16 high byte
        find_crc_low,        // Enhanced mode only: CRC-16 low byte
        find_end_byte
    };
    _fsm _state;

    uint8_t  _idByte;
    uint8_t  _bytesToRec;
    uint8_t  _payIndex;
    uint64_t _packetStart;  // ms since boot; 64-bit so it never wraps (32-bit wrapped after 49.7 days)
    uint32_t _timeout;
    uint8_t  _recCrcHigh;  // Enhanced mode: stored CRC-16 high byte

    // --- Callbacks ---

    const stCallbackPtr *_callbacks;
    uint8_t _callbacksLen;

    // --- CRC-8 (compatible mode, polynomial 0x9B) ---

    uint8_t _crcTable8[256];
    void    _generateCrcTable8(uint8_t polynomial = 0x9B);
    uint8_t _calcCrc8(uint8_t *data, uint8_t len);

    // --- CRC-16 (enhanced mode, CRC-16/CCITT-FALSE, polynomial 0x1021) ---

    uint16_t _crcTable16[256];
    void     _generateCrcTable16(uint16_t polynomial = 0x1021);
    uint16_t _calcCrc16(uint8_t *data, uint8_t len);

    // --- COBS encoding ---

    void    _calcOverhead(uint8_t *data, uint8_t len);
    int16_t _findLast(uint8_t *data, uint8_t len);
    void    _stuffPacket(uint8_t *data, uint8_t len);
    void    _unpackPacket(uint8_t *data);

    // --- Packet operations ---

    uint8_t _sendBytes(const void *data, uint16_t len, uint8_t packetID);
    uint8_t _transmitLocked(uint16_t messageLen, uint8_t packetID);
    uint8_t _constructPacket(uint16_t messageLen, uint8_t packetID);
    uint8_t _parse(uint8_t recChar, bool valid);
    void    _resetParser();

    // --- TX mutex (statically allocated, see constructor) ---

    StaticSemaphore_t _txMutexBuf;
    SemaphoreHandle_t _txMutex;
};
