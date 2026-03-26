/**
 * @file    SEN0244.cpp
 * @brief   SEN0244 TDS/Conductivity sensor driver implementation
 */

#include "Sen0244.h"
#include <math.h>  

// Constructor  

SEN0244::SEN0244(uint8_t adcPin)
    : _adcPin(adcPin),
      _lastError(SEN0244_OK),
      _bufferIndex(0),
      _bufferFull(false)
{
    // Initialise sample buffer to zero
    for (int i = 0; i < SEN0244_SAMPLE_COUNT; i++) {
        _analogBuffer[i] = 0;
    }
}

// Public API  

void SEN0244::begin() {
    pinMode(_adcPin, INPUT);
    // Set 12-bit ADC resolution (ESP32-S3 supports up to 12-bit)
    analogReadResolution(12);
}


SEN0244Data SEN0244::read(float temperature) {
    SEN0244Data result;
    result.tds_ppm       = 0.0f;
    result.voltage_V     = 0.0f;
    result.temperature_C = temperature;
    result.valid         = false;

    // Add one new sample to the rolling buffer
    _analogBuffer[_bufferIndex] = analogRead(_adcPin);
    _bufferIndex++;
    if (_bufferIndex >= SEN0244_SAMPLE_COUNT) {
        _bufferIndex = 0;
        _bufferFull  = true;
    }

    // Work on a copy of the buffer (median sort is destructive)
    int tempBuffer[SEN0244_SAMPLE_COUNT];
    for (int i = 0; i < SEN0244_SAMPLE_COUNT; i++) {
        tempBuffer[i] = _analogBuffer[i];
    }

    // Don't report a valid reading until buffer has filled at least once
    if (!_bufferFull) {
        _lastError = SEN0244_OK;  
        return result;            
    }

    // Compute median ADC value and convert to voltage
    int medianADC = _getMedian(tempBuffer, SEN0244_SAMPLE_COUNT);
    float rawVoltage = medianADC * (SEN0244_VREF / (float)SEN0244_ADC_MAX);

    // Sanity check: SEN0244 max output is 2.3V
    if (rawVoltage > 2.5f) {
        _lastError = SEN0244_ERR_RANGE;
        return result;
    }

    // Temperature compensation
    float compensationCoeff   = 1.0f + 0.02f * (temperature - 25.0f);
    float compensatedVoltage  = rawVoltage / compensationCoeff;

    // Convert voltage to TDS using DFRobot polynomial
    float tds = _voltageToPPM(compensatedVoltage);

    // Clamp to valid range
    if (tds < 0.0f)    tds = 0.0f;
    if (tds > 1000.0f) tds = 1000.0f;

    result.tds_ppm   = tds;
    result.voltage_V = compensatedVoltage;
    result.valid     = true;
    _lastError       = SEN0244_OK;

    return result;
}


int SEN0244::readRaw() {
    return analogRead(_adcPin);
}


float SEN0244::readVoltage() {
    return analogRead(_adcPin) * (SEN0244_VREF / (float)SEN0244_ADC_MAX);
}


void SEN0244::resetBuffer() {
    for (int i = 0; i < SEN0244_SAMPLE_COUNT; i++) {
        _analogBuffer[i] = 0;
    }
    _bufferIndex = 0;
    _bufferFull  = false;
}


// Private helpers 

int SEN0244::_getMedian(int* buffer, int len) {
    // Bubble sort (acceptable for 30 samples)
    for (int j = 0; j < len - 1; j++) {
        for (int i = 0; i < len - j - 1; i++) {
            if (buffer[i] > buffer[i + 1]) {
                int temp      = buffer[i];
                buffer[i]     = buffer[i + 1];
                buffer[i + 1] = temp;
            }
        }
    }
    // Return middle value
    if ((len & 1) > 0) {
        return buffer[(len - 1) / 2];
    } else {
        return (buffer[len / 2] + buffer[len / 2 - 1]) / 2;
    }
}


float SEN0244::_voltageToPPM(float voltage) {
    // DFRobot empirical cubic polynomial
    return (133.42f * pow(voltage, 3.0f)
          - 255.86f * pow(voltage, 2.0f)
          + 857.39f * voltage) * 0.5f;
}