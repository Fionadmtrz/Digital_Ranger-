/**
 * @file    DFR0198.h
 * @brief   Driver for DFRobot DFR0198 — Waterproof DS18B20 water temperature sensor
 */

#pragma once

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//Error codes 
#define DFR0198_OK                0
#define DFR0198_ERR_NO_DEVICE    -1   // no sensor found on 1-Wire bus
#define DFR0198_ERR_DISCONNECTED -2   // sensor found but not responding
#define DFR0198_ERR_CRC          -3   // CRC check failed (noisy cable)

// Sentinel value returned on error
#define DFR0198_INVALID_TEMP     -127.0f

// Data structure 

struct DFR0198Data {
    float   temperature_C;  ///< Water temperature in °C (−55 to +125°C)
    bool    valid;         
    uint8_t sensorCount;    
};

// Driver class  

class DFR0198 {
public:
    /**
     * @brief Construct a DFR0198 driver                
     */
    explicit DFR0198(uint8_t dataPin);

    /**
     * @brief Initialise the sensor
     * @return DFR0198_OK on success
     */
    int8_t begin();

    /**
     * @brief Request and read temperature from the first sensor on the bus
     */
    DFR0198Data read();

    /**
     * @brief Read temperature from a specific sensor by index
     */
    DFR0198Data readByIndex(uint8_t index);

    /**
     * @brief Set resolution: Default is 12
     */
    void setResolution(uint8_t bits);

    /**
     * @brief Return how many DS18B20 sensors were found during begin()
     */
    uint8_t sensorCount() const { return _sensorCount; }

    /**
     * @brief Return the last error code
     */
    int8_t lastError() const { return _lastError; }

    /**
     * @brief Return true if begin() succeeded
     */
    bool isReady() const { return _ready; }

private:
    uint8_t           _dataPin;
    OneWire           _oneWire;
    DallasTemperature _sensors;

    bool    _ready;
    int8_t  _lastError;
    uint8_t _sensorCount;
    uint8_t _resolution;
};