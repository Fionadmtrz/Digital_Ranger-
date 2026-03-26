/**
 * @file    bme280_driver.h
 * @brief   Driver for Bosch BME280 — air temperature, humidity, and pressure sensor
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// I²C address 
// Default: SDO tied to GND → 0x76
// Change to 0x77 if SDO is tied to VDDIO
#define BME280_I2C_ADDR_DEFAULT  0x76
#define BME280_I2C_ADDR_ALT      0x77

// Register addresses (BME280 datasheet §5.3) 
#define BME280_REG_ID            0xD0
#define BME280_REG_RESET         0xE0
#define BME280_REG_CTRL_HUM      0xF2
#define BME280_REG_STATUS        0xF3
#define BME280_REG_CTRL_MEAS     0xF4
#define BME280_REG_CONFIG        0xF5
#define BME280_REG_PRESS_MSB     0xF7
#define BME280_REG_CALIB_00      0x88    
#define BME280_REG_CALIB_26      0xE1    

//Register values 
#define BME280_CHIP_ID           0x60
#define BME280_RESET_VALUE       0xB6

// Oversampling settings (datasheet §5.4.3 / §5.4.5)
#define BME280_OSRS_SKIP         0x00   
#define BME280_OSRS_1            0x01   
#define BME280_OSRS_2            0x02
#define BME280_OSRS_4            0x03
#define BME280_OSRS_8            0x04
#define BME280_OSRS_16           0x05

// Mode bits (mode[1:0] in ctrl_meas, datasheet §5.4.5)
#define BME280_MODE_SLEEP        0x00
#define BME280_MODE_FORCED       0x01   // trigger one measurement
#define BME280_MODE_NORMAL       0x03

// IIR filter (filter[2:0] in config, datasheet §5.4.6)
#define BME280_FILTER_OFF        0x00
#define BME280_FILTER_2          0x01
#define BME280_FILTER_4          0x02
#define BME280_FILTER_8          0x03
#define BME280_FILTER_16         0x04

// t_sb standby time 
#define BME280_STANDBY_0_5MS     0x00
#define BME280_STANDBY_62_5MS    0x01
#define BME280_STANDBY_125MS     0x02
#define BME280_STANDBY_250MS     0x03
#define BME280_STANDBY_500MS     0x04
#define BME280_STANDBY_1000MS    0x05
#define BME280_STANDBY_10MS      0x06
#define BME280_STANDBY_20MS      0x07

// Error codes 
#define BME280_OK                 0
#define BME280_ERR_NOT_FOUND     -1   // chip not detected on I²C bus
#define BME280_ERR_WRONG_ID      -2   // wrong chip ID (not 0x60)
#define BME280_ERR_TIMEOUT       -3   // measurement did not complete in time
#define BME280_ERR_I2C           -4   // I²C communication error


//Data structure 

/**
 * @brief Compensated output from a single BME280 measurement
 */
struct BME280Data {
    float temperature_C;    ///< Air temperature in °C        
    float humidity_pct;     ///< Relative humidity in %RH     
    float pressure_hPa;     ///< Barometric pressure in hPa  
    bool  valid;            ///< true if measurement succeeded
};


// Driver class 

class BME280Driver {
public:
    /**
     * @brief Construct a BME280Driver.
     * @param address  I²C address 
     * @param wire     TwoWire instance (default Wire)
     */
    explicit BME280Driver(uint8_t address = BME280_I2C_ADDR_DEFAULT,
                          TwoWire* wire   = &Wire);

    /**
     * @brief Initialise the sensor: verify chip ID, read calibration data,
     *        and configure for forced-mode operation.
     * @return BME280_OK on success
     */
    int8_t begin();

    /**
     * @brief Trigger one forced-mode measurement and return compensated results.
     */
    BME280Data read();

    /**
     * @brief Perform a soft reset (write 0xB6 to reset register).
     */
    void reset();

    /**
     * @brief Return the last error code from begin() or read().
     */
    int8_t lastError() const { return _lastError; }

    /**
     * @brief Return true if the sensor was successfully initialised.
     */
    bool isReady() const { return _ready; }

private:
    // I²C helpers
    bool     _writeReg(uint8_t reg, uint8_t value);
    bool     _readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t  _readReg(uint8_t reg);

    // Calibration data readout (datasheet §4.2.2)
    void _readCalibration();

    // Compensation formulas (datasheet §4.2.3)
    float _compensateTemperature(int32_t adc_T);
    float _compensateHumidity(int32_t adc_H);
    float _compensatePressure(int32_t adc_P);

    // Wait for measurement to complete by polling status register
    bool _waitMeasurementReady(uint32_t timeoutMs = 100);

    // Configuration
    uint8_t   _address;
    TwoWire*  _wire;

    // State
    bool    _ready;
    int8_t  _lastError;

    // Calibration coefficients (datasheet §4.2.2, Table 16)
    // Temperature
    uint16_t _dig_T1;
    int16_t  _dig_T2, _dig_T3;
    // Pressure
    uint16_t _dig_P1;
    int16_t  _dig_P2, _dig_P3, _dig_P4, _dig_P5;
    int16_t  _dig_P6, _dig_P7, _dig_P8, _dig_P9;
    // Humidity
    uint8_t  _dig_H1;
    int16_t  _dig_H2;
    uint8_t  _dig_H3;
    int16_t  _dig_H4, _dig_H5;
    int8_t   _dig_H6;

    // t_fine: shared between temperature and pressure/humidity compensation
    int32_t  _t_fine;
};