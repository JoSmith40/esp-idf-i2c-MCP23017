| Supported Targets | ESP32-C2/C3 | ESP32-C5/C6 | ESP32-H2 | ESP32-P4 | ESP32-S2/S3 | T-Display-S3/...S3-AMOLED |
| ----------------- |  ---------- | ----------- | -------- | -------- | ----------- | ------------------------- |

## Overview
Use this program to check if your Port expander device is still alive.
This is a demonstration application for the ESP32 in the ESP-IDF environment V5.4 with the newer I2C-API `i2c_master`

This code has been updated and compiled with the following versions:
- VSCode: March 2025 (version 1.99.1)
- ESP-IDF 5.4.1
- CMake version 3.30.2

## Hardware Required
1 x ESP32 board<br>
1 x I2C port expander MCP23017<br>
2 x 4K7 resistors<br>

The MCP23017 is an I2C bus GPIO expander.

Thank an NSBum: (https://github.com/NSBum/esp32-mcp23017-demo.git)

## How to use this project
Run command below in your terminal.
