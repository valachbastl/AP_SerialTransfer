# AP_SerialTransfer

Packet-based UART communication library for ESP-IDF, compatible with Arduino SerialTransfer protocol.

## Features

- Packet-based UART communication with COBS encoding (Consistent Overhead Byte Stuffing)
- Two operating modes: **enhanced** (default) and **arduino-compat**
- Enhanced mode: CRC-16/CCITT-FALSE, sequence numbers for lost/duplicate packet detection
- Arduino-compat mode: CRC-8 (polynomial 0x9B), full compatibility with Arduino SerialTransfer
- Template methods `txObj()`, `rxObj()`, `sendDatum()` for sending/receiving any data type
- Non-blocking receive via `available()`
- Callback support mapped to packet ID
- Thread-safe sending (internal mutex) - safe to call `sendData()` from multiple RTOS tasks
- Configurable timeout for stale packet detection
- ESP-IDF logging (ESP_LOGI, ESP_LOGW, ESP_LOGE)

## Installation

### PlatformIO

Add to `platformio.ini`:

```ini
lib_deps =
    https://github.com/valachbastl/AP_SerialTransfer.git
```

Or with specific version:

```ini
lib_deps =
    https://github.com/valachbastl/AP_SerialTransfer.git#v1.0.0
```

## Usage

### Basic Setup

```cpp
#include "AP_SerialTransfer.h"

AP_SerialTransfer transfer("myTag");
transfer.begin(UART_NUM_1, /* TX */ 7, /* RX */ 8);
```

### Sending a Struct

```cpp
struct SensorData {
    float temperature;
    float humidity;
};

SensorData data = { 23.5f, 65.0f };
transfer.sendDatum(data);  // send as single packet
```

### Receiving a Struct

```cpp
if (transfer.available())
{
    SensorData data = {};
    transfer.rxObj(data);
    // use data.temperature, data.humidity
}
```

### Multi-Object Packets

```cpp
// Sending
uint16_t offset = 0;
offset = transfer.txObj(temperature, offset);
offset = transfer.txObj(humidity, offset);
transfer.sendData(offset);

// Receiving
if (transfer.available())
{
    uint16_t offset = 0;
    float temperature, humidity;
    offset = transfer.rxObj(temperature, offset);
    offset = transfer.rxObj(humidity, offset);
}
```

### Arduino-Compatible Mode

```cpp
AP_SerialTransfer transfer("myTag", true);  // enable compat mode
transfer.begin(UART_NUM_1, 7, 8);
// Now compatible with Arduino SerialTransfer on the other end
```

### Callbacks

```cpp
void onSensorData() { /* handle packet ID 0 */ }
void onCommand()    { /* handle packet ID 1 */ }

const stCallbackPtr callbacks[] = { onSensorData, onCommand };
transfer.setCallbacks(callbacks, 2);

// In loop:
transfer.tick();  // dispatches callbacks on received packets
```

## Packet Format

### Enhanced (default)

```
START | ID | SEQ | COBS | LEN | payload | CRC16_H | CRC16_L | STOP
 0x7E   1B   1B    1B    1B    N bytes     1B         1B      0x81
```

### Arduino-compat

```
START | ID | COBS | LEN | payload | CRC8 | STOP
 0x7E   1B   1B    1B    N bytes    1B    0x81
```

## API Reference

### Constructor

| Method | Description |
|--------|-------------|
| `AP_SerialTransfer(tag, arduinoCompatible)` | Constructor (arduinoCompatible default false) |

### Instance Methods

| Method | Description |
|--------|-------------|
| `begin(uartNum, txPin, rxPin, baudRate, rxBufSize)` | Initialize UART (baudRate default 115200, rxBufSize default 1024) |
| `sendData(messageLen, packetID)` | Send txBuff contents as packet (packetID default 0) |
| `available()` | Non-blocking receive, returns payload bytes count (0 = no data) |
| `tick()` | Wrapper for available(), dispatches callbacks |
| `currentPacketID()` | Get last received packet ID |
| `currentSeqNum()` | Get last received sequence number (enhanced mode only) |
| `reset()` | Reset buffers and parser state |
| `setCallbacks(callbacks, count)` | Set callback array mapped to packet IDs |
| `setTimeout(timeoutMs)` | Set stale packet timeout (default 50ms) |

### Template Methods

| Method | Description |
|--------|-------------|
| `txObj(val, index, len)` | Write object to TX buffer at index (default 0) |
| `rxObj(val, index, len)` | Read object from RX buffer at index (default 0) |
| `sendDatum(val, len)` | Send single object as packet (shortcut for txObj + sendData) |

### Public Members

| Member | Description |
|--------|-------------|
| `txBuff[254]` | TX buffer |
| `rxBuff[254]` | RX buffer |
| `bytesRead` | Bytes received in last packet |
| `status` | Last operation status code |

### Status Codes

| Code | Value | Description |
|------|-------|-------------|
| `ST_NEW_DATA` | 2 | New packet received |
| `ST_NO_DATA` | 1 | No data available |
| `ST_CRC_ERROR` | 0 | CRC mismatch |
| `ST_PAYLOAD_ERROR` | -1 | Invalid payload length |
| `ST_STOP_BYTE_ERROR` | -2 | Missing stop byte |
| `ST_STALE_PACKET_ERROR` | -3 | Packet reception timed out |
| `ST_CONTINUE` | 3 | Parsing in progress |

## Author

Petr Adamek
