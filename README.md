# esphome_i2c_sniffer – ESPHome External Component

[![ESPHome](https://img.shields.io/badge/ESPHome-compatible-blue?logo=esphome)](https://esphome.io/)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-integration-41BDF5?logo=home-assistant)](https://www.home-assistant.io/)
[![ESPHome](https://img.shields.io/badge/ESPHome-2024.x%2B-blue)](https://esphome.io)
[![Platform](https://img.shields.io/badge/Platform-ESP32-red)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/Framework-ESP--IDF-green)](https://docs.espressif.com/projects/esp-idf/)
 
> **Passive I²C Bus Sniffer for ESPHome**
> Decodes I²C traffic on any GPIO pins and exposes it as sensors, logs, and automation callbacks — without interfering with the bus.
 
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
- Optional **numeric sensors** for the last seen 7-bit device address and last data byte
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
| Multi-block transactions | Register-Read (repeated START) merged into a single output line |
| Last-address sensor | Optional numeric sensor exposing the most recently seen 7-bit address |
| Last-data sensor | Optional numeric sensor exposing the most recently seen data byte |
| `on_address` callback | Trigger ESPHome automations based on captured I²C address events |
| ESPHome log integration | Every transaction logged at DEBUG level |
| ESPHome external component | Installed directly from Git — no manual file copying required |
 
---
 
## How It Works
 
The component monitors the GPIO lines assigned to SDA and SCL using interrupt-driven edge detection:
 
- **Falling edge on SDA while SCL is HIGH** → START condition detected
- **Rising edge on SDA while SCL is HIGH** → STOP condition detected
- **Rising edge on SCL** → data bit sampled from SDA
 
Each START/STOP-delimited transaction is decoded into:
- The **7-bit device address**
- The **read/write direction** (R or W)
- All **data bytes** exchanged
- The **ACK/NACK** status per byte (indicated by `+` for ACK, `-` for NACK)
 
Combined transactions using a **repeated START** (e.g., a register-write followed by a data-read, common in many sensors) are detected and merged into a single output line for readability.
 
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
  - source:
      type: git
      url: https://github.com/GernotAlthammer/esphome_i2c_sniffer
      ref: main
    components: [ esphome_i2c_sniffer ]
```
 
No manual file downloads or copies are required. ESPHome will fetch the component automatically during compilation.
 
---
 
## Configuration
 
### Minimal Example
 
This minimal configuration sets up the sniffer on GPIO 16 (SDA) and GPIO 17 (SCL) and publishes decoded transactions as a text sensor:
 
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/GernotAlthammer/esphome_i2c_sniffer
      ref: main
    components: [ esphome_i2c_sniffer ]
 
esphome:
  name: i2c-sniffer
  friendly_name: I2C Sniffer
 
esp32:
  board: esp32dev
  framework:
    type: arduino
 
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
 
api:
ota:
logger:
  level: DEBUG
 
esphome_i2c_sniffer:
  sda: GPIO16
  scl: GPIO17
 
text_sensor:
  - platform: esphome_i2c_sniffer
    name: "I2C Transaction"
```
 
### Full Example with Optional Sensors
 
This example additionally exposes the last-seen I²C address and last data byte as numeric sensors, and uses the `on_address` callback to trigger an action whenever a specific device (e.g., `0x54`) is detected on the bus:
 
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/GernotAlthammer/esphome_i2c_sniffer
      ref: main
    components: [ esphome_i2c_sniffer ]
 
esphome:
  name: i2c-sniffer
  friendly_name: I2C Sniffer
 
esp32:
  board: esp32dev
  framework:
    type: arduino
 
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
 
api:
ota:
logger:
  level: DEBUG
 
esphome_i2c_sniffer:
  sda: GPIO16
  scl: GPIO17
  on_address:
    - address: 0x54
      then:
        - logger.log: "Detected device at address 0x54!"
 
text_sensor:
  - platform: esphome_i2c_sniffer
    name: "I2C Transaction"
 
sensor:
  - platform: esphome_i2c_sniffer
    last_address:
      name: "I2C Last Address"
    last_data:
      name: "I2C Last Data Byte"
```
 
### Configuration Options Reference
 
#### `esphome_i2c_sniffer` block
 
| Option | Type | Required | Description |
|---|---|---|---|
| `sda` | Pin | **Yes** | GPIO pin connected to the SDA line of the monitored bus |
| `scl` | Pin | **Yes** | GPIO pin connected to the SCL line of the monitored bus |
| `on_address` | Automation list | No | Callback triggered when a specific I²C address is seen on the bus |
 
#### `text_sensor` platform: `esphome_i2c_sniffer`
 
| Option | Type | Required | Description |
|---|---|---|---|
| `name` | String | **Yes** | Entity name for the decoded transaction text sensor in Home Assistant |
 
#### `sensor` platform: `esphome_i2c_sniffer`
 
| Option | Type | Required | Description |
|---|---|---|---|
| `last_address` | Sensor config | No | Numeric sensor (0–127) for the most recently captured 7-bit I²C address |
| `last_data` | Sensor config | No | Numeric sensor (0–255) for the most recently captured data byte |
 
---
 
## Output Format
 
Each captured I²C transaction is published to the text sensor as a single line in the following format:
 
```
[<timestamp_ms>ms] [<address>]<direction>(<data_byte>)<ack> [| [<address>]<direction>(<data_byte>)<ack> ...]
```
 
| Field | Meaning |
|---|---|
| `<timestamp_ms>` | Milliseconds since device boot when the transaction was captured |
| `<address>` | 7-bit I²C device address in hex (e.g., `0x54`) |
| `<direction>` | `W` = Write, `R` = Read |
| `(<data_byte>)` | Hex byte value, e.g., `(10)` for 0x10 |
| `<ack>` | `+` = ACK received, `-` = NACK received |
| `\|` | Separator between segments of a repeated-START (combined) transaction |
 
**Example output:**
 
```
[40563ms] [0x54]W+(10)+ | [0x54]R+(00)+(00)-
```
 
This represents a combined register-read transaction:
1. A Write to device `0x54`, sending register address `0x10` (ACK'd)
2. Followed by a Read from `0x54`, receiving bytes `0x00`, `0x00` (last byte NACK'd, indicating end of read)
 
---
 
## Home Assistant Integration
 
Once the device is running and connected to Home Assistant via the ESPHome API, the following entities will appear automatically:
 
- **Text sensor** (`sensor.<name>_i2c_transaction`) — updates with every decoded I²C transaction
- **Numeric sensor** (`sensor.<name>_i2c_last_address`) — the last seen 7-bit device address (optional)
- **Numeric sensor** (`sensor.<name>_i2c_last_data_byte`) — the last seen data byte (optional)
 
You can use these entities in **Lovelace dashboards**, **History graphs**, or **Home Assistant automations** to react to specific I²C activity.
 
---
 
## Automations and Callbacks
 
The `on_address` callback is triggered each time a specified I²C device address appears in a captured transaction. This allows the ESPHome device to react to I²C activity using any ESPHome automation action.
 
**Example: Log a message when address 0x3C (common OLED display) is seen:**
 
```yaml
esphome_i2c_sniffer:
  sda: GPIO16
  scl: GPIO17
  on_address:
    - address: 0x3C
      then:
        - logger.log: "OLED display communication detected (0x3C)"
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
        └──────── SDA → GPIO16 (ESP32)
        └──────── SCL → GPIO17 (ESP32)
                          │
                    [I2C Sniffer ESP32]
```
 
**Important safety notes:**
 
- The GPIO pins must be configured as **inputs only**. The component handles this internally.
- If the monitored bus operates at **3.3 V** and your ESP32 also runs at 3.3 V, direct connection is generally safe.
- If there is a **voltage mismatch** (e.g., 5 V I²C bus), use a **bidirectional I²C isolator** (e.g., ISO1541, ADUM1250) or a level-shifter with the isolator to protect your ESP32.
- An **I²C bus isolator** (e.g., ISO154x series) is strongly recommended when sniffing live production systems, as it prevents accidental bus interference and protects both devices.
 
---
 
## Project Structure
 
```
esphome_i2c_sniffer/
├── components/
│   └── esphome_i2c_sniffer/
│       ├── __init__.py          # ESPHome component registration (Python)
│       ├── i2c_sniffer.h        # C++ component header
│       └── i2c_sniffer.cpp      # C++ component implementation
└── README.md
```
 
The component is implemented in **C++ (≈78%)** for performance-critical interrupt handling and bus decoding, with a **Python (≈22%)** layer for ESPHome YAML validation and code generation.
 
---
 
## Limitations
 
- **High-frequency buses**: The ESP32 is fast enough for standard-mode (100 kHz) and fast-mode (400 kHz) I²C. Fast-mode Plus (1 MHz) or higher may result in missed bits.
- **Continuous high-volume traffic**: If the I²C bus is very busy and transactions arrive faster than the component can publish them, some transactions may be dropped. This is inherent to a software-based bit-banging sniffer.
- **No bus driving**: The sniffer cannot inject data or simulate ACK/NACK responses. It is a read-only observer.
- **ESP8266 not recommended**: Due to the lower clock speed of the ESP8266, it may miss bits at standard 100 kHz I²C bus speeds. Use an ESP32 for reliable results.
 
---
 
## Disclaimer
 
This project is a hobby project. It is provided **as-is**, without any warranty of any kind — express or implied — including but not limited to fitness for a particular purpose or merchantability.
 
Use at your own risk. Connecting your ESP32 to a live I²C bus without proper isolation may damage your hardware. The author is not associated with any company and assumes no liability for any harm arising from the use of this software.
 
If you find this project useful or want to improve it, feel free to fork and contribute!
 
---
 
*Maintained by [GernotAlthammer](https://github.com/GernotAlthammer)*
 
