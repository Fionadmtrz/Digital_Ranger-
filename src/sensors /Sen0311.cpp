/**
 * @file    SEN0311.cpp
 * @brief   SEN0311 (A02YYUW) waterproof ultrasonic sensor driver implementation
 */

#include "Sen0311.h"

//Constructor  

SEN0311::SEN0311(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin)
    : _serial(serial),
      _rxPin(rxPin),
      _txPin(txPin),
      _ready(false),
      _lastError(SEN0311_OK)
{}

// Public API  

void SEN0311::begin() {
    // ESP32-S3 HardwareSerial.begin() with explicit RX/TX pins
    _serial.begin(SEN0311_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
    delay(100);  // allow sensor to stabilise after power-up
    flush();     // discard any stale bytes in the buffer
    _ready = true;
}


SEN0311Data SEN0311::read() {
    SEN0311Data result;
    result.distance_mm = 0;
    result.distance_cm = 0.0f;
    result.valid       = false;

    if (!_ready) {
        _lastError = SEN0311_ERR_TIMEOUT;
        return result;
    }

    // Flush stale data 
    flush();

    uint8_t  buf[SEN0311_PACKET_SIZE];
    uint32_t startTime = millis();

    while ((millis() - startTime) < SEN0311_READ_TIMEOUT_MS) {

        // Wait for the 0xFF header byte
        if (!_serial.available()) {
            delay(5);
            continue;
        }

        uint8_t incoming = _serial.read();
        if (incoming != SEN0311_HEADER) {
            continue;  
        }

        // Header found 
        uint32_t packetStart = millis();
        while (_serial.available() < 3) {
            if ((millis() - packetStart) > 50) {
                // Timeout waiting for rest of packet
                _lastError = SEN0311_ERR_TIMEOUT;
                return result;
            }
            delay(2);
        }

        buf[0] = SEN0311_HEADER;
        buf[1] = _serial.read();  // H — distance high byte
        buf[2] = _serial.read();  // L — distance low byte
        buf[3] = _serial.read();  // SUM — checksum

        uint16_t distance_mm;
        if (!_parsePacket(buf, distance_mm)) {
            // Checksum failed 
            _lastError = SEN0311_ERR_CHECKSUM;
            continue;
        }

        // Check blind zone 
        if (distance_mm < SEN0311_MIN_DISTANCE_MM) {
            _lastError = SEN0311_ERR_BLIND_ZONE;
            return result;
        }

        // Check maximum range 
        if (distance_mm > SEN0311_MAX_DISTANCE_MM) {
            _lastError = SEN0311_ERR_OUT_OF_RANGE;
            return result;
        }

        // Valid reading
        result.distance_mm = distance_mm;
        result.distance_cm = distance_mm / 10.0f;
        result.valid       = true;
        _lastError         = SEN0311_OK;
        return result;
    }

    // Reached timeout without a valid packet
    _lastError = SEN0311_ERR_TIMEOUT;
    return result;
}


void SEN0311::flush() {
    // Discard all bytes currently in the UART receive buffer
    while (_serial.available()) {
        _serial.read();
    }
}


// Private helpers 

bool SEN0311::_parsePacket(uint8_t* buf, uint16_t& distanceOut) {
    // Verify checksum: SUM = (0xFF + H + L) & 0xFF
    uint8_t expectedSum = (buf[0] + buf[1] + buf[2]) & 0xFF;
    if (buf[3] != expectedSum) {
        return false;
    }

    // Reconstruct distance: H * 256 + L
    distanceOut = ((uint16_t)buf[1] << 8) | buf[2];
    return true;
}