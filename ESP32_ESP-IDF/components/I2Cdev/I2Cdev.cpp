// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// EFM32 stub port by Nicolas Baldeck <nicolas@pioupiou.fr>
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// ESP32/IDF port improvements:
//      - readBytes: Implemented proper single-transaction Repeated START.
//      - writeBytes: Added guard for length=1 cases to prevent undefined behavior.
//      - readWord: Corrected return value to reflect bytes read.
//      - Error handling: Enhanced reporting and buffer safety.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "I2Cdev.h"

#define I2C_NUM         I2C_NUM_0
#define I2C_TIMEOUT_MS  100   // I2C işlem timeout'u (ms)

static const char *TAG = "I2Cdev";

/** Default constructor. */
I2Cdev::I2Cdev() {}

/** Stub – I2C init example.cpp içinde yapılıyor. */
void I2Cdev::initialize() {}
void I2Cdev::enable(bool isEnabled) {}

/** Default timeout value for read operations. */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

// ─── OKUMA FONKSİYONLARI ─────────────────────────────────────────────────────

/**
 * @brief Read multiple bytes from a device register.
 *
 * Uses a single I2C transaction with Repeated START for correct bus behavior.
 * Sequence: START → addr+W → regAddr → rSTART → addr+R → [data bytes] → STOP
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    if (length == 0) return 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Phase 1: write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);

    // Phase 2: repeated START, then read
    i2c_master_start(cmd);  // Repeated START – no STOP between phases
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, true);

    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    // Clear caller buffer on error for safety
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "readBytes failed (dev=0x%02X reg=0x%02X len=%d err=0x%x)",
                 devAddr, regAddr, length, ret);
        memset(data, 0, length);
        return 0;
    }

    return length;
}

/** Read a single byte. */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/**
 * @brief Read a single bit from an 8-bit device register.
 * Note: *data is non-zero (not necessarily 1) when the bit is set.
 * Callers should test (data != 0), not (data == 1).
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b = 0;
    int8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/**
 * @brief Read multiple bits from an 8-bit device register.
 * Result is right-aligned: bits [bitStart..bitStart-length+1] → bits [length-1..0].
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    uint8_t b = 0;
    int8_t count = readByte(devAddr, regAddr, &b, timeout);
    if (count != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/**
 * @brief Read two bytes (big-endian) from device registers.
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    uint8_t buf[2] = {0, 0};
    int8_t count = readBytes(devAddr, regAddr, 2, buf, timeout);
    *data = (uint16_t)((buf[0] << 8) | buf[1]);
    return count;  // 2 on success, 0 on failure
}

// ─── YAZMA FONKSİYONLARI ─────────────────────────────────────────────────────

/**
 * @brief Write a single byte to a device register.
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "writeByte failed (dev=0x%02X reg=0x%02X err=0x%x)", devAddr, regAddr, ret);
        return false;
    }
    return true;
}

/**
 * @brief Write multiple bytes to a device register.
 *
 * Optimized to handle single-byte writes safely.
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    if (length == 0) return true;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);

    if (length > 1) {
        // write first (length-1) bytes without ACK check on each
        i2c_master_write(cmd, data, length - 1, false);
    }
    // write last byte with ACK check
    i2c_master_write_byte(cmd, data[length - 1], true);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "writeBytes failed (dev=0x%02X reg=0x%02X len=%d err=0x%x)",
                 devAddr, regAddr, length, ret);
        return false;
    }
    return true;
}

/** Write two bytes (big-endian) to a device register. */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    uint8_t buf[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    return writeBytes(devAddr, regAddr, 2, buf);
}

/**
 * @brief Write a single bit in an 8-bit device register (read-modify-write).
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b = 0;
    if (readByte(devAddr, regAddr, &b) == 0) return false;
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/**
 * @brief Write multiple bits in an 8-bit device register (read-modify-write).
 * Data should be right-aligned.
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t b = 0;
    if (readByte(devAddr, regAddr, &b) == 0) return false;

    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    b &= ~mask;
    b |= data;
    return writeByte(devAddr, regAddr, b);
}

// SelectRegister is no longer needed – readBytes handles the combined transaction.
// Kept as a no-op stub in case any external code references it.
void I2Cdev::SelectRegister(uint8_t dev, uint8_t reg) {
    // Intentionally empty – merged into readBytes as a Repeated START.
    (void)dev; (void)reg;
}
