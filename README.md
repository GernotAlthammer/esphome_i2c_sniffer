# esphome_i2c_sniffer – ESPHome External Component

[![ESPHome](https://img.shields.io/badge/ESPHome-compatible-blue?logo=esphome)](https://esphome.io/)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-integration-41BDF5?logo=home-assistant)](https://www.home-assistant.io/)
[![ESPHome](https://img.shields.io/badge/ESPHome-2024.x%2B-blue)](https://esphome.io)
[![Platform](https://img.shields.io/badge/Platform-ESP32-red)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/Framework-ESP--IDF-green)](https://docs.espressif.com/projects/esp-idf/)

> **Passive I²C Bus Sniffer for ESPHome**
> Decodes I²C traffic on any GPIO pins and exposes it as sensors, logs, and automation callbacks — without interfering with the bus.

---

## Changelog

**2.1.0**
- Fixed: the address token in the published message was previously emitted as `0x54` (with a `0x` prefix), which is *not* a valid 2-hex-digit token and was therefore silently dropped by any downstream parser that only keeps 2-hex-digit tokens — shifting every subsequent data-byte index by one. The address is now emitted as a plain `54`, consistent with the data bytes.
- Changed: `address` (the `on_address` trigger argument, `last_addr_sensor`, and the address token in the message) now consistently means the **raw 8-bit byte as seen on the wire** — 7-bit device address shifted left, OR'd with the R/W bit in bit 0 — rather than the bare 7-bit address. E.g. a write to 7-bit address `0x40` is reported as `0x80`. This matches how many real-world I²C captures/tools report addresses, and lets the low bit double as an inline read/write indicator.
- Fixed: `last_byte_sensor` was declared and wired up in the schema but never actually published to. It now publishes the value of the last data byte in each transaction.
- Added: per-byte ACK/NACK tracking (address byte included), surfaced as a trailing `ACK:...` token that never collides with the 2-hex-digit byte tokens.
- Added: millisecond timestamp (captured at the START condition) as the first token of each published message.
- Known gap (unchanged from before): repeated-START transactions are not yet merged into a single output line — see [How It Works](#how-it-works).

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [How It Works](#how-it-works)
- [Typical Use Cases](#typical-use-cases)
- [Requirements](#requirements)
- [Installation](#installation)
- [Configuration](#configuration)
  - [Minimal Example](#minimal-example)
  - [Full Example with Optional Sensors](#full-example-with-optional-sensors)
  - [Configuration Options Reference](#configuration-options-reference)
- [Output Format](#output-format)
- [Home Assistant Integration](#home-assistant-integration)
- [Automations and Callbacks](#automations-and-callbacks)
- [Hardware Wiring Notes](#hardware-wiring-notes)
- [Project Structure](#project-structure)
- [Limitations](#limitations)
- [Disclaimer](#disclaimer)

---

## Overview

`esphome_i2c_sniffer` is an ESPHome **external component** that passively listens on any two GPIO pins configured as SDA and SCL, decodes all I²C bus transactions in real time, and makes the results available as:

- A **text sensor** publishing each decoded transaction (with timestamp)
- Optional **numeric sensors** for the last seen address byte and last data byte
- **ESPHome log output** (DEBUG level) for every captured message
- An **`on_address` callback** for use in automations or advanced event-driven logic

The sniffer is **strictly passive**: it never drives the SDA or SCL lines, never sends ACK/NACK signals, and does not interfere with the monitored bus in any way.

---

## Features

| Feature | Details |
|---|---|
| Passive sniffing | Pure input-only — never drives the I²C bus |
| Any GPIO pins | SDA and SCL can be freely assigned to any suitable GPIO |
| Text sensor output | Each transaction published as a human-readable timestamped string |
| Per-byte ACK/NACK | Address byte and every data byte carry their own ACK/NACK status |
| Last-address sensor | Optional numeric sensor exposing the most recently seen address byte (see [Changelog](#changelog) for what "address" means) |
| Last-data sensor | Optional numeric sensor exposing the data-byte count of the most recently seen transaction |
| Last-byte sensor | Optional numeric sensor exposing the value of the last data byte of the most recently seen transaction |
| `on_address` callback | Trigger ESPHome automations based on captured I²C address events |
| ESPHome log integration | Every transaction logged at DEBUG level |
| ESPHome external component | Installed directly from Git — no manual file copying required |

---

## How It Works

The component monitors the GPIO lines assigned to SDA and SCL using interrupt-driven edge detection:

- **Falling edge on SDA while SCL is HIGH** → START condition detected
- **Rising edge on SDA while SCL is HIGH** → STOP condition detected
- **Rising edge on SCL** → data bit sampled from SDA (and, on the 9th clock, the ACK/NACK bit)

Each START/STOP-delimited transaction is decoded into:
- The **raw 8-bit address byte** (7-bit address + R/W bit, see [Changelog](#changelog)) and its ACK/NACK
- The **read/write direction** (R or W), derived from the same byte's low bit
- All **data bytes** exchanged, each with its own ACK/NACK
- A **millisecond timestamp** taken at the START condition

> **Note on repeated START:** a repeated START (e.g. a register-pointer write immediately followed by a data read, without an intervening STOP) is currently decoded as if it were a fresh transaction — the first segment before the repeated START is not merged into the same output line. If you need that merged, let us know / open an issue; it's a planned enhancement, not yet implemented.

---

## Typical Use Cases

- **Reverse engineering** unknown I²C devices or proprietary protocols
- **Debugging** I²C communication between a microcontroller and a peripheral
- **Sniffing DDC/CI or EDID** traffic on HDMI/DVI connections
- **Monitoring EEPROM read/write** operations
- **Analyzing sensor communication** between a host system and I²C sensors
- **Observing I²C traffic** between two third-party devices without modifying either

---

## Requirements

- An **ESP32** (recommended due to its processing speed at higher I²C bus frequencies)  
  > ⚠️ ESP8266 may work for low-speed buses but is generally too slow for 400 kHz I²C
- **ESPHome** (any recent version supporting `external_components`)
- **Two free GPIO pins** to connect to the SDA and SCL lines of the bus being monitored
- Optional: an **I²C bus isolator** (e.g., ISO1541, ADUM1250) for electrical safety when sniffing live devices

---

## Installation

Add the component to your ESPHome YAML configuration using the `external_components` directive, pointing directly at this repository:

```yaml
external_components:
  - source: github://GernotAlthammer/esphome_i2c_sniffer
    components: [ esphome_i2c_sniffer ]
    refresh: 1d
```

No manual file downloads or copies are required. ESPHome will fetch the component automatically during compilation.

Note: The standard interval the source will be checked by ESPHome is 1d (1 day). You can make ESPHome check the repository every time by setting this option to 0s, however since ESPHome is validating the configuration continuously while using the dashboard or the vscode extension, it is not recommended to set this value to less than a few minutes to avoid validation slow down and excessive repository checks.

---

## Configuration

### Minimal Example

This minimal configuration sets up the sniffer on GPIO 18 (SDA) and GPIO 19 (SCL) and publishes decoded transactions as a text sensor:

```yaml
external_components:
  - source: github://GernotAlthammer/esphome_i2c_sniffer
    components: [ esphome_i2c_sniffer ]
    refresh: 1d

esphome:
  name: i2c-sniffer
  friendly_name: I2C Sniffer

esp32:
  board: esp32dev
  framework:
    type: esp-idf

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
ota:
logger:
  level: DEBUG

esphome_i2c_sniffer:
  id: i2c_sniffer 
  scl_pin: 19 
  sda_pin: 18 

  msg_sensor:
    name: "I2C Message"
```

### Full Example with Optional Sensors

This example additionally exposes the last-seen address byte and last data byte as numeric sensors, and uses the `on_address` callback to trigger an action whenever a specific device/direction (e.g., a write to wire-byte `0x80`) is detected on the bus:

```yaml
external_components:
  - source: github://GernotAlthammer/esphome_i2c_sniffer
    components: [ esphome_i2c_sniffer ]
    refresh: 1d

esphome:
  name: i2c-sniffer
  friendly_name: I2C Sniffer

esp32:
  board: esp32dev
  framework:
    type: esp-idf

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
ota:
logger:
  level: DEBUG

esphome_i2c_sniffer:
  id: i2c_sniffer 
  scl_pin: 19
  sda_pin: 18

  last_data_sensor:
    name: "I2C Last Data"

  last_byte_sensor:
    name: "I2C Last Byte"

  last_addr_sensor:
    name: "I2C Last Addr"

  on_address:
    - lambda: |-
        // 'address' is available here automatically (raw 8-bit wire byte)
        if (address == 0x80 ) {
           ESP_LOGD("I2C_Data", "I2C data detected on 0x80");
        }

  msg_sensor:
    id: i2c_message_internal
    internal: true

    on_value:
      - then:
          - lambda: |-
              std::string msg = x;
              std::stringstream ss(msg);
              std::string byte_str;
              std::vector<int> bytes;
              std::string decimal_output = "";

              while (ss >> byte_str) {
                if (byte_str.size() == 2 && isxdigit((unsigned char) byte_str[0]) && isxdigit((unsigned char) byte_str[1])) {
                  int val = strtol(byte_str.c_str(), nullptr, 16);
                  bytes.push_back(val);
                  if (!decimal_output.empty()) decimal_output += ", ";
                  decimal_output += std::to_string(val);
                }
              }
              if (!bytes.empty() ) {
                ESP_LOGD("I2C_Data", "Decimal: [%s]", decimal_output.c_str());
              }
```

### Configuration Options Reference

#### `esphome_i2c_sniffer` block

| Option | Type | Required | Description |
|---|---|---|---|
| `sda_pin` | uint8 GPIO number | **Yes** | GPIO pin connected to the SDA line of the monitored bus |
| `scl_pin` | uint8 GPIO number | **Yes** | GPIO pin connected to the SCL line of the monitored bus |
| `id` | ID | **Yes** (auto-generated if omitted) | Entity ID for the component instance |
| `msg_sensor` | Text_sensor config | No | Text sensor for the most recently captured message |
| `last_data_sensor` | Sensor config | No | Numeric sensor: number of data bytes in the most recently captured transaction |
| `last_byte_sensor` | Sensor config | No | Numeric sensor (0–255) for the value of the last data byte in the most recently captured transaction |
| `last_addr_sensor` | Sensor config | No | Numeric sensor (0–255) for the raw address byte of the most recently captured transaction |
| `on_address` | Automation list | No | Callback triggered when a specific address byte is seen on the bus. Arguments available in the lambda: `address` (uint8, raw wire byte) and `rw` (bool, true = read) |

---

## Output Format

Each captured I²C transaction is published to the text sensor as a single line in the following format:

```
<timestamp_ms>ms <address_hex> <R|W> <data_byte_1> <data_byte_2> ... ACK:<addr_ack><byte1_ack><byte2_ack>...
```

| Field | Meaning |
|---|---|
| `<timestamp_ms>` | Milliseconds since device boot when the START condition was captured |
| `<address_hex>` | The raw 8-bit address byte as seen on the wire (7-bit address + R/W bit), as a plain two-digit hex token, e.g. `80` — deliberately **without** a `0x` prefix or any suffix, so that every address and data token in the line is a uniform, unambiguous two-hex-digit value |
| `<R\|W>` | `R` = Read, `W` = Write (redundant with bit 0 of the address byte, included for readability) |
| `<data_byte_N>` | Data bytes as plain two-digit hex tokens, in the order received, e.g. `10` for `0x10` |
| `ACK:...` | One `+`/`-` character per byte (address first, then each data byte in order); `+` = ACK, `-` = NACK. This trailing token is intentionally never a valid 2-hex-digit token, so parsers that only keep whitespace-separated 2-hex-digit tokens (e.g. `while (ss >> tok) if (tok is 2 hex digits) ...`) can safely ignore it while still picking up the address and every data byte, in order, starting at index 0 |

**Example output:**

```
40563ms 80 W 10 ACK:+++
```

This represents: a Write (address byte `0x80`, ACK'd), sending data byte `0x10` (ACK'd).

**Parsing tip:** since the address and every data byte are uniform, un-prefixed, un-suffixed two-hex-digit tokens, and every other token in the line (timestamp, direction, ACK summary) is deliberately *not* a two-hex-digit token, this simple ESPHome lambda reconstructs them in order:

```cpp
std::string msg = x;  // x = the msg_sensor's new value
std::stringstream ss(msg);
std::string tok;
std::vector<int> bytes;  // bytes[0] = address byte, bytes[1..] = data bytes, in order
while (ss >> tok) {
  if (tok.size() == 2 && isxdigit((unsigned char) tok[0]) && isxdigit((unsigned char) tok[1])) {
    bytes.push_back((int) strtol(tok.c_str(), nullptr, 16));
  }
}
```

---

## Home Assistant Integration

Once the device is running and connected to Home Assistant via the ESPHome API, the following entities will appear automatically (for whichever optional sensors you configured):

- **Message sensor** — updates with every decoded I²C transaction
- **Numeric sensor** (`last_addr_sensor`) — the last seen address byte (optional)
- **Numeric sensor** (`last_data_sensor`) — the data-byte count of the last transaction (optional)
- **Numeric sensor** (`last_byte_sensor`) — the last seen data byte value (optional)

You can use these entities in **Lovelace dashboards**, **History graphs**, or **Home Assistant automations** to react to specific I²C activity.

---

## Automations and Callbacks

The `on_address` callback is triggered each time a matching address byte appears in a captured transaction. This allows the ESPHome device to react to I²C activity using any ESPHome automation action.

**Example: Log a message when a write to 7-bit address 0x3C (common OLED display) is seen — on the wire that's byte 0x78:**

```yaml
esphome_i2c_sniffer:
  id: i2c_sniffer 
  scl_pin: 19 
  sda_pin: 18
  on_address:
    - lambda: |-
        // 'address' is available here automatically (raw 8-bit wire byte)
        if (address == 0x78 ) {
           ESP_LOGD("I2C_Data", "OLED display communication detected (0x3C, write)");
        }
```

Multiple `on_address` entries with different addresses can be listed in sequence.

---

## Hardware Wiring Notes

Connect the sniffer GPIO pins **in parallel** to the SDA and SCL lines of the bus you want to monitor:

```
Monitored Device (Master)         Target Device (Slave)
        |                                 |
       SDA ──────────────────────────── SDA
       SCL ──────────────────────────── SCL
        |                                 |
        └──────── SDA → GPIO18 (ESP32)
        └──────── SCL → GPIO19 (ESP32)
                          │
                    [I2C Sniffer ESP32]
```

**Important safety notes:**

- The GPIO pins must be configured as **inputs only**. The component handles this internally.
- If the monitored bus operates at **3.3 V** and your ESP32 also runs at 3.3 V, direct connection is generally safe.
- If there is a **voltage mismatch** (e.g., 5 V I²C bus), use a **bidirectional I²C isolator** (e.g., ISO1541, ADUM1250) or a level-shifter with the isolator to protect your ESP32. Since this component only ever reads the lines, a simple resistive voltage divider on SDA/SCL is also a workable, cheaper option for passive listening specifically — it would not be sufficient for anything that needs to drive the bus.
- An **I²C bus isolator** (e.g., ISO154x series) is strongly recommended when sniffing live production systems, as it prevents accidental bus interference and protects both devices.

---

## Project Structure

```
esphome_i2c_sniffer/
├── components/
│   └── esphome_i2c_sniffer/
│       ├── __init__.py                # ESPHome component registration (Python)
│       ├── esphome_i2c_sniffer.h      # C++ component header
│       └── esphome_i2c_sniffer.cpp    # C++ component implementation
└── README.md
```

The component is implemented in **C++** for performance-critical interrupt handling and bus decoding, with a **Python** layer for ESPHome YAML validation and code generation.

---

## Limitations

- **High-frequency buses**: The ESP32 is fast enough for standard-mode (100 kHz) and fast-mode (400 kHz) I²C. Fast-mode Plus (1 MHz) or higher may result in missed bits.
- **Continuous high-volume traffic**: If the I²C bus is very busy and transactions arrive faster than the component can publish them, some transactions may be dropped. This is inherent to a software-based bit-banging sniffer.
- **No bus driving**: The sniffer cannot inject data or simulate ACK/NACK responses. It is a read-only observer.
- **ESP8266 not recommended**: Due to the lower clock speed of the ESP8266, it may miss bits at standard 100 kHz I²C bus speeds. Use an ESP32 for reliable results.
- **No repeated-START merging yet**: see the note in [How It Works](#how-it-works).

---

## Disclaimer

This project is a hobby project. It is provided **as-is**, without any warranty of any kind — express or implied — including but not limited to fitness for a particular purpose or merchantability.

Use at your own risk. Connecting your ESP32 to a live I²C bus without proper isolation may damage your hardware. The author is not associated with any company and assumes no liability for any harm arising from the use of this software.

If you find this project useful or want to improve it, feel free to fork and contribute!

---

*Maintained by [GernotAlthammer](https://github.com/GernotAlthammer)*
