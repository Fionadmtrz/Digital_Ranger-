/**
 * @file    DFR0198.cpp
 * @brief   DFR0198 (DS18B20) waterproof temperature sensor driver implementation
 *
 * Uses blocking conversion — requestTemperatures() waits up to 750ms
 * for the DS18B20 to complete conversion at 12-bit resolution.
 * This is intentional: 750ms is negligible compared to our 5–15 min
 * sleep interval, and non-blocking would add unnecessary complexity.
 *
 * Error detection relies on DallasTemperature returning DEVICE_DISCONNECTED_C
 * (-127.0°C) when CRC fails or sensor is not responding.
 */

#include "DS18B20.h"

//Constructor 
DFR0198::DFR0198(uint8_t dataPin)
    : _dataPin(dataPin),
      _oneWire(dataPin),
      _sensors(&_oneWire),
      _ready(false),
      _lastError(DFR0198_OK),
      _sensorCount(0),
      _resolution(12)
{}

// Public API 
int8_t DFR0198::begin() {
    _ready = false;

    // Initialise the DallasTemperature library
    _sensors.begin();

    // Count sensors on the 1-Wire bus
    _sensorCount = _sensors.getDeviceCount();

    if (_sensorCount == 0) {
        _lastError = DFR0198_ERR_NO_DEVICE;
        return _lastError;
    }

    // Set 12-bit resolution on all sensors
    // Resolution is stored in sensor EEPROM — survives power cycles
    _sensors.setResolution(_resolution);

    // Use blocking conversion — library waits for conversion to complete
    // before returning from requestTemperatures()
    _sensors.setWaitForConversion(true);

    _ready = true;
    _lastError = DFR0198_OK;
    return DFR0198_OK;
}


DFR0198Data DFR0198::read() {
    return readByIndex(0);
}


DFR0198Data DFR0198::readByIndex(uint8_t index) {
    DFR0198Data result;
    result.temperature_C = DFR0198_INVALID_TEMP;
    result.valid         = false;
    result.sensorCount   = _sensorCount;

    if (!_ready) {
        _lastError = DFR0198_ERR_NO_DEVICE;
        return result;
    }

    // Request temperature conversion from all sensors on the bus
    // With setWaitForConversion(true), this blocks for up to 750ms
    _sensors.requestTemperatures();

    // Read compensated temperature from sensor at given index
    float tempC = _sensors.getTempCByIndex(index);

    // DallasTemperature returns -127.0 if sensor disconnected or CRC failed
    if (tempC == DEVICE_DISCONNECTED_C) {
        _lastError = DFR0198_ERR_DISCONNECTED;
        return result;
    }

    // Sanity check — DS18B20 physical range is -55 to +125°C
    // In wetland context we expect 0–40°C; flag anything outside sensor spec
    if (tempC < -55.0f || tempC > 125.0f) {
        _lastError = DFR0198_ERR_CRC;
        return result;
    }

    result.temperature_C = tempC;
    result.valid         = true;
    _lastError           = DFR0198_OK;
    return result;
}


void DFR0198::setResolution(uint8_t bits) {
    // Clamp to valid DS18B20 resolution range
    if (bits < 9)  bits = 9;
    if (bits > 12) bits = 12;
    _resolution = bits;

    if (_ready) {
        _sensors.setResolution(_resolution);
    }
}