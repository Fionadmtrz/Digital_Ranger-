/**
 * @file    bme280_driver.h
 * @brief   Driver for Bosch BME280 — air temperature, humidity, and pressure sensor
 *
 * Sensor:    BME280 (AST1025 card variant)
 * Interface: I²C
 * Address:   0x76 (SDO → GND, default) or 0x77 (SDO → VDDIO)
 * Platform:  ESP32-S3, PlatformIO / Arduino framework
 *
 * Operating mode: FORCED MODE — one measurement on demand, then returns to
 * sleep. Ideal for duty-cycled low-power deployments (target: ~1 sample / 5–15 min).
 *
 * Key datasheet constraints to keep in mind:
 *   - NOT waterproof: board must stay inside the sealed enclosure
 *   - Sensitive to condensation — ensure a vented Gore-Tex or similar membrane
 *   - Keep away from heat sources (ESP32-S3 self-heating effect on temperature)
 *   - I²C address 0x76 conflicts with MS5837-02BA if ever added in v2
 *   - Full accuracy temperature range: 0–65 °C (operational: −40 to +85 °C)
 *
 * Typical current in forced mode (osrs x1 / x1 / x1, filter off):
 *   ~0.16 µA average at 1 sample/min  (datasheet §3.5.1)
 *
 * References:
 *   Bosch BME280 Datasheet Rev 1.6, BST-BME280-DS002-15
 *   Adafruit BME280 Arduino library (compensation formulas cross-checked)
 *
 * Usage:
 *   BME280Driver bme;
 *   bme.begin();
 *   BME280Data d = bme.read();
 *   Serial.println(d.temperature_C);
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// I²C address 
// Default: SDO tied to GND → 0x76
// Change to 0x77 if SDO is tied to VDDIO (and you need to avoid MS5837 conflict in v2)
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
#define BME280_REG_CALIB_00      0x88   // T1..P9 calibration start
#define BME280_REG_CALIB_26      0xE1   // H2..H6 calibration start

//Register values 
#define BME280_CHIP_ID           0x60
#define BME280_RESET_VALUE       0xB6

// Oversampling settings (osrs field values, datasheet §5.4.3 / §5.4.5)
#define BME280_OSRS_SKIP         0x00   // skip measurement
#define BME280_OSRS_1            0x01   // oversampling ×1
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

// t_sb standby time (for normal mode only — not used in forced mode)
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
 * @brief Compensated output from a single BME280 measurement.
 *
 * All values are set to NAN if the measurement fails.
 */
struct BME280Data {
    float temperature_C;    ///< Air temperature in °C       (accuracy ±1.0 °C, 0–65 °C)
    float humidity_pct;     ///< Relative humidity in %RH    (accuracy ±3 %RH)
    float pressure_hPa;     ///< Barometric pressure in hPa  (accuracy ±1.0 hPa, 0–65 °C)
    bool  valid;            ///< true if measurement succeeded
};


// Driver class 

class BME280Driver {
public:
    /**
     * @brief Construct a BME280Driver.
     * @param address  I²C address (default 0x76). Use 0x77 if SDO tied to VDDIO.
     * @param wire     TwoWire instance (default Wire). Pass Wire1 for a second I²C bus.
     */
    explicit BME280Driver(uint8_t address = BME280_I2C_ADDR_DEFAULT,
                          TwoWire* wire   = &Wire);

    /**
     * @brief Initialise the sensor: verify chip ID, read calibration data,
     *        and configure for forced-mode operation.
     *
     * Call once in setup(). Wire.begin() must be called before this.
     *
     * @return BME280_OK on success, negative error code on failure.
     */
    int8_t begin();

    /**
     * @brief Trigger one forced-mode measurement and return compensated results.
     *
     * Blocks for the measurement duration (~10 ms with osrs ×1).
     * Returns a BME280Data with valid=false on any error.
     */
    BME280Data read();

    /**
     * @brief Perform a soft reset (write 0xB6 to reset register).
     *
     * Useful to recover from a locked-up state. Call begin() again after reset.
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

    // Compensation formulas (datasheet §4.2.3 — integer 32/64-bit path)
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