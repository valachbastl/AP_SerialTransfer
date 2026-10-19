#include "AP_SerialTransfer.h"


AP_SerialTransfer::AP_SerialTransfer(const char *tag, bool arduinoCompatible)
    : _tag(tag),
      _uartNum(UART_NUM_1),
      _initialized(false),
      _arduinoCompat(arduinoCompatible),
      _overheadByte(0),
      _recOverheadByte(0),
      _txSeqNum(0),
      _rxSeqNum(0),
      _state(find_start_byte),
      _idByte(0),
      _bytesToRec(0),
      _payIndex(0),
      _packetStart(0),
      _timeout(50),
      _recCrcHigh(0),
      _callbacks(nullptr),
      _callbacksLen(0),
      bytesRead(0),
      status(0)
{
    // Static allocation — cannot fail (no OOM), the handle is valid for the
    // whole object lifetime, even if begin() fails or is never called.
    _txMutex = xSemaphoreCreateMutexStatic(&_txMutexBuf);

    memset(txBuff, 0, sizeof(txBuff));
    memset(rxBuff, 0, sizeof(rxBuff));
    memset(_preamble, 0, sizeof(_preamble));
    memset(_postamble, 0, sizeof(_postamble));

    _preamble[0] = ST_START_BYTE;

    if (_arduinoCompat) {
        _preambleSize  = ST_PREAMBLE_SIZE_COMPAT;
        _postambleSize = ST_POSTAMBLE_SIZE_COMPAT;
        _postamble[1]  = ST_STOP_BYTE;
        _generateCrcTable8();
    } else {
        _preambleSize  = ST_PREAMBLE_SIZE_ENHANCED;
        _postambleSize = ST_POSTAMBLE_SIZE_ENHANCED;
        _postamble[2]  = ST_STOP_BYTE;
        _generateCrcTable16();
    }
}


esp_err_t AP_SerialTransfer::begin(uart_port_t uartNum, int txPin, int rxPin,
                                   int baudRate, size_t rxBufSize)
{
    if (_initialized) return ESP_OK;  // idempotent

    _uartNum = uartNum;

    uart_config_t uartConfig = {};
    uartConfig.baud_rate  = baudRate;
    uartConfig.data_bits  = UART_DATA_8_BITS;
    uartConfig.parity     = UART_PARITY_DISABLE;
    uartConfig.stop_bits  = UART_STOP_BITS_1;
    uartConfig.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(_uartNum, rxBufSize, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(_tag, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    // Driver is installed from here on — uninstall it on any later failure so
    // begin() can be retried (a second uart_driver_install() would fail).
    err = uart_param_config(_uartNum, &uartConfig);
    if (err != ESP_OK) {
        ESP_LOGE(_tag, "uart_param_config failed: %s", esp_err_to_name(err));
        uart_driver_delete(_uartNum);
        return err;
    }

    err = uart_set_pin(_uartNum, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(_tag, "uart_set_pin failed: %s", esp_err_to_name(err));
        uart_driver_delete(_uartNum);
        return err;
    }

    _initialized = true;
    ESP_LOGI(_tag, "UART%d initialized (TX:%d, RX:%d, baud:%d, mode:%s)",
             _uartNum, txPin, rxPin, baudRate,
             _arduinoCompat ? "arduino-compat" : "enhanced");
    return ESP_OK;
}


uint8_t AP_SerialTransfer::sendData(uint16_t messageLen, uint8_t packetID)
{
    if (!_initialized) return 0;

    xSemaphoreTake(_txMutex, portMAX_DELAY);
    uint8_t numBytes = _transmitLocked(messageLen, packetID);
    xSemaphoreGive(_txMutex);

    return numBytes;
}


// Copy + send under one mutex hold — the whole send is atomic, so concurrent
// sendDatum()/sendPacket() calls from different tasks cannot overwrite each
// other's txBuff contents.
uint8_t AP_SerialTransfer::_sendBytes(const void *data, uint16_t len, uint8_t packetID)
{
    if (!_initialized) return 0;
    if (len > ST_MAX_PACKET_SIZE) len = ST_MAX_PACKET_SIZE;

    xSemaphoreTake(_txMutex, portMAX_DELAY);
    memcpy(txBuff, data, len);
    uint8_t numBytes = _transmitLocked(len, packetID);
    xSemaphoreGive(_txMutex);

    return numBytes;
}


// Caller must hold _txMutex.
uint8_t AP_SerialTransfer::_transmitLocked(uint16_t messageLen, uint8_t packetID)
{
    uint8_t numBytes = _constructPacket(messageLen, packetID);

    uart_write_bytes(_uartNum, _preamble, _preambleSize);
    uart_write_bytes(_uartNum, txBuff, numBytes);
    uart_write_bytes(_uartNum, _postamble, _postambleSize);

    return numBytes;
}


uint8_t AP_SerialTransfer::available()
{
    if (!_initialized) return 0;

    uint8_t recChar = 0xFF;
    bool valid = false;

    size_t buffered = 0;
    uart_get_buffered_data_len(_uartNum, &buffered);

    if (buffered > 0)
    {
        valid = true;

        while (buffered > 0)
        {
            int len = uart_read_bytes(_uartNum, &recChar, 1, 0);
            if (len <= 0) break;

            bytesRead = _parse(recChar, valid);

            // Error paths reset the parser themselves (_parse), nothing to do here.
            if (status != ST_CONTINUE)
                break;

            uart_get_buffered_data_len(_uartNum, &buffered);
        }
    }
    else
    {
        bytesRead = _parse(recChar, false);
    }

    return bytesRead;
}


bool AP_SerialTransfer::tick()
{
    if (available())
        return true;
    return false;
}


uint8_t AP_SerialTransfer::currentPacketID()
{
    return _idByte;
}


uint8_t AP_SerialTransfer::currentSeqNum()
{
    return _rxSeqNum;
}


void AP_SerialTransfer::reset()
{
    if (!_initialized) return;

    uart_flush_input(_uartNum);

    // Clears RX only (parser state). txBuff belongs to the TX path guarded by
    // _txMutex — touching it here would race with sendData() and drop outgoing data.
    memset(rxBuff, 0, sizeof(rxBuff));

    _resetParser();
}


// Parser-only reset used on framing errors: no UART flush, so packets already
// buffered behind a corrupted one survive. The parser then hunts for the next
// start byte; a false start (0x7E can also appear inside a payload or header)
// is rejected by the length, CRC and stop byte checks.
void AP_SerialTransfer::_resetParser()
{
    bytesRead    = 0;
    _packetStart = 0;
    _state       = find_start_byte;
}


void AP_SerialTransfer::setCallbacks(const stCallbackPtr *callbacks, uint8_t count)
{
    _callbacks    = callbacks;
    _callbacksLen = count;
}


void AP_SerialTransfer::setTimeout(uint32_t timeoutMs)
{
    _timeout = timeoutMs;
}


// --- CRC-8 (polynomial 0x9B, compatible with Arduino SerialTransfer) ---

void AP_SerialTransfer::_generateCrcTable8(uint8_t polynomial)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        int curr = i;
        for (int j = 0; j < 8; j++)
        {
            if ((curr & 0x80) != 0)
                curr = (curr << 1) ^ (int)polynomial;
            else
                curr <<= 1;
        }
        _crcTable8[i] = (uint8_t)curr;
    }
}


uint8_t AP_SerialTransfer::_calcCrc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc = _crcTable8[crc ^ data[i]];
    return crc;
}


// --- CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF) ---

void AP_SerialTransfer::_generateCrcTable16(uint16_t polynomial)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint16_t curr = i << 8;
        for (int j = 0; j < 8; j++)
        {
            if ((curr & 0x8000) != 0)
                curr = (curr << 1) ^ polynomial;
            else
                curr <<= 1;
        }
        _crcTable16[i] = curr;
    }
}


uint16_t AP_SerialTransfer::_calcCrc16(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
        crc = (_crcTable16[((crc >> 8) ^ data[i]) & 0xFF]) ^ (crc << 8);
    return crc;
}


// --- COBS encoding (Consistent Overhead Byte Stuffing) ---

void AP_SerialTransfer::_calcOverhead(uint8_t *data, uint8_t len)
{
    _overheadByte = 0xFF;

    for (uint8_t i = 0; i < len; i++)
    {
        if (data[i] == ST_START_BYTE)
        {
            _overheadByte = i;
            break;
        }
    }
}


int16_t AP_SerialTransfer::_findLast(uint8_t *data, uint8_t len)
{
    for (uint8_t i = (len - 1); i != 0xFF; i--)
        if (data[i] == ST_START_BYTE)
            return i;
    return -1;
}


void AP_SerialTransfer::_stuffPacket(uint8_t *data, uint8_t len)
{
    int16_t refByte = _findLast(data, len);

    if (refByte != -1)
    {
        for (uint8_t i = (len - 1); i != 0xFF; i--)
        {
            if (data[i] == ST_START_BYTE)
            {
                data[i] = refByte - i;
                refByte = i;
            }
        }
    }
}


void AP_SerialTransfer::_unpackPacket(uint8_t *data)
{
    // COBS unstuffing bounded by the payload length (_bytesToRec) — protects
    // against writes past rxBuff on corrupted/malicious COBS pointers from the wire.
    uint8_t testIndex = _recOverheadByte;
    uint8_t delta = 0;

    while (testIndex < _bytesToRec && data[testIndex])
    {
        delta = data[testIndex];
        data[testIndex] = ST_START_BYTE;
        testIndex += delta;
    }

    if (testIndex < _bytesToRec)
        data[testIndex] = ST_START_BYTE;
}


// --- Packet construction ---

uint8_t AP_SerialTransfer::_constructPacket(uint16_t messageLen, uint8_t packetID)
{
    uint8_t len = (messageLen > ST_MAX_PACKET_SIZE) ? ST_MAX_PACKET_SIZE : (uint8_t)messageLen;

    _calcOverhead(txBuff, len);
    _stuffPacket(txBuff, len);

    if (_arduinoCompat)
    {
        // Compat: START | ID | COBS | LEN | payload | CRC8 | STOP
        uint8_t crcVal = _calcCrc8(txBuff, len);

        _preamble[1] = packetID;
        _preamble[2] = _overheadByte;
        _preamble[3] = len;

        _postamble[0] = crcVal;
    }
    else
    {
        // Enhanced: START | ID | SEQ | COBS | LEN | payload | CRC16_H | CRC16_L | STOP
        uint16_t crcVal = _calcCrc16(txBuff, len);

        _preamble[1] = packetID;
        _preamble[2] = _txSeqNum++;  // Auto-increment the sequence number
        _preamble[3] = _overheadByte;
        _preamble[4] = len;

        _postamble[0] = (uint8_t)(crcVal >> 8);    // High byte
        _postamble[1] = (uint8_t)(crcVal & 0xFF);  // Low byte
    }

    return len;
}


// --- Parser state machine ---

uint8_t AP_SerialTransfer::_parse(uint8_t recChar, bool valid)
{
    uint64_t now = esp_timer_get_time() / 1000;
    bool packetFresh = (_packetStart == 0) || ((now - _packetStart) < _timeout);

    if (!packetFresh)
    {
        ESP_LOGW(_tag, "STALE_PACKET_ERROR");
        _resetParser();

        // No byte to process (poll without data) → report the timeout. Otherwise
        // fall through: the byte that revealed the timeout may be the start of a
        // new packet and must not be dropped.
        if (!valid)
        {
            status = ST_STALE_PACKET_ERROR;
            return bytesRead;
        }
    }

    if (valid)
    {
        switch (_state)
        {
        case find_start_byte:
        {
            if (recChar == ST_START_BYTE)
            {
                _state       = find_id_byte;
                _packetStart = now;
            }
            break;
        }

        case find_id_byte:
        {
            _idByte = recChar;
            // Compat → overhead byte next, Enhanced → sequence number first
            _state = _arduinoCompat ? find_overhead_byte : find_seq_byte;
            break;
        }

        case find_seq_byte:
        {
            // Enhanced mode only
            _rxSeqNum = recChar;
            _state    = find_overhead_byte;
            break;
        }

        case find_overhead_byte:
        {
            _recOverheadByte = recChar;
            _state           = find_payload_len;
            break;
        }

        case find_payload_len:
        {
            if ((recChar > 0) && (recChar <= ST_MAX_PACKET_SIZE))
            {
                _bytesToRec = recChar;
                _payIndex   = 0;
                _state      = find_payload;
            }
            else
            {
                ESP_LOGW(_tag, "PAYLOAD_ERROR");
                _resetParser();
                status = ST_PAYLOAD_ERROR;
                return bytesRead;
            }
            break;
        }

        case find_payload:
        {
            if (_payIndex < _bytesToRec)
            {
                rxBuff[_payIndex] = recChar;
                _payIndex++;

                if (_payIndex == _bytesToRec)
                    _state = find_crc;
            }
            break;
        }

        case find_crc:
        {
            if (_arduinoCompat)
            {
                // Compat: single-byte CRC-8
                uint8_t calcCrc = _calcCrc8(rxBuff, _bytesToRec);

                if (calcCrc == recChar)
                {
                    _state = find_end_byte;
                }
                else
                {
                    ESP_LOGW(_tag, "CRC_ERROR");
                    _resetParser();
                    status = ST_CRC_ERROR;
                    return bytesRead;
                }
            }
            else
            {
                // Enhanced: first CRC-16 byte (high byte), store and continue
                _recCrcHigh = recChar;
                _state      = find_crc_low;
            }
            break;
        }

        case find_crc_low:
        {
            // Enhanced mode: second CRC-16 byte (low byte)
            uint16_t recCrc  = ((uint16_t)_recCrcHigh << 8) | recChar;
            uint16_t calcCrc = _calcCrc16(rxBuff, _bytesToRec);

            if (calcCrc == recCrc)
            {
                _state = find_end_byte;
            }
            else
            {
                ESP_LOGW(_tag, "CRC_ERROR (CRC-16: expected 0x%04X, got 0x%04X)", calcCrc, recCrc);
                _resetParser();
                status = ST_CRC_ERROR;
                return bytesRead;
            }
            break;
        }

        case find_end_byte:
        {
            _state = find_start_byte;

            if (recChar == ST_STOP_BYTE)
            {
                _unpackPacket(rxBuff);
                bytesRead = _bytesToRec;
                status    = ST_NEW_DATA;

                if (_callbacks)
                {
                    if (_idByte < _callbacksLen)
                        _callbacks[_idByte]();
                    else
                        ESP_LOGW(_tag, "No callback for packet ID %d", _idByte);
                }

                _packetStart = 0;
                return _bytesToRec;
            }

            ESP_LOGW(_tag, "STOP_BYTE_ERROR");
            _resetParser();
            status = ST_STOP_BYTE_ERROR;

            // Truncated frame followed directly by a new one: this byte may be
            // the next start byte — begin the new packet instead of dropping it.
            if (recChar == ST_START_BYTE)
            {
                _state       = find_id_byte;
                _packetStart = now;
            }
            return bytesRead;
        }

        default:
        {
            ESP_LOGW(_tag, "Undefined parser state %d", _state);
            _resetParser();
            break;
        }
        }
    }
    else
    {
        bytesRead = 0;
        status    = ST_NO_DATA;
        return bytesRead;
    }

    bytesRead = 0;
    status    = ST_CONTINUE;
    return bytesRead;
}
