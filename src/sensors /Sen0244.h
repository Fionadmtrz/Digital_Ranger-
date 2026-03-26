/**
 * @file    SEN0244.h
 * @brief   Driver for DFRobot SEN0244 — Gravity Analog TDS / Conductivity Sensor
 */

#pragma once

#include <Arduino.h>

//Configuration 
#define SEN0244_VREF          3.3f    // ADC reference voltage 
#define SEN0244_ADC_MAX       4095    // 12-bit ADC max value
#define SEN0244_SAMPLE_COUNT  30      // number of samples for median filter
#define SEN0244_DEFAULT_TEMP  25.0f   // fallback temperature if none provided

// Error codes 
#define SEN0244_OK            0
#define SEN0244_ERR_NO_ADC   -1   // ADC pin not readable
#define SEN0244_ERR_RANGE    -2   // reading out of expected range

// Data structure 

struct SEN0244Data {
    float   tds_ppm;         ///< TDS in ppm (0–1000 ppm range)
    float   voltage_V;       ///< Raw compensated voltage before TDS conversion
    float   temperature_C;   ///< Temperature used for compensation
    bool    valid;           ///< true if reading is valid
};

// Driver class 

class SEN0244 {
public:
    /**
     * @brief Construct a SEN0244 driver
     */
    explicit SEN0244(uint8_t adcPin);

    /**
     * @brief Initialise the sensor
     */
    void begin();

    /**
     * @brief Take a filtered TDS reading with temperature compensation.
     
     * @param temperature  Water temperature in °C from DS18B20 (DFR0198).
     *                     Defaults to 25°C if not provided.
     */
    SEN0244Data read(float temperature = SEN0244_DEFAULT_TEMP);

    /**
     * @brief Read raw ADC value (0–4095) without filtering or compensation for debugging
     */
    int readRaw();

    /**
     * @brief Read raw voltage (0–3.3V)
     */
    float readVoltage();

    /**
     * @brief Reset the internal sample buffer
     */
    void resetBuffer();

    /**
     * @brief Return the last error code
     */
    int8_t lastError() const { return _lastError; }

private:
    uint8_t _adcPin;
    int8_t  _lastError;

    // Rolling sample buffer for median filter
    int     _analogBuffer[SEN0244_SAMPLE_COUNT];
    int     _bufferIndex;
    bool    _bufferFull;

    // Median filter 
    int _getMedian(int* buffer, int len);

    // Convert compensated voltage to TDS using DFRobot polynomial
    float _voltageToPPM(float voltage);
};