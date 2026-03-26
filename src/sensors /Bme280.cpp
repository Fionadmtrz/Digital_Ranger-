/**
 * @file    bme280_driver.cpp
 * @brief   BME280 driver implementation — forced mode, integer compensation
 */

#include "Bme280.h"
#include <math.h>   // for NAN

// Constructor  

BME280Driver::BME280Driver(uint8_t address, TwoWire* wire)
    : _address(address), _wire(wire),
      _ready(false), _lastError(BME280_OK),
      _dig_T1(0), _dig_T2(0), _dig_T3(0),
      _dig_P1(0), _dig_P2(0), _dig_P3(0), _dig_P4(0), _dig_P5(0),
      _dig_P6(0), _dig_P7(0), _dig_P8(0), _dig_P9(0),
      _dig_H1(0), _dig_H2(0), _dig_H3(0), _dig_H4(0), _dig_H5(0), _dig_H6(0),
      _t_fine(0)
{}


// Public API 

int8_t BME280Driver::begin() {
    _ready = false;

    // 1. Verify the sensor responds on the I²C bus
    _wire->beginTransmission(_address);
    if (_wire->endTransmission() != 0) {
        _lastError = BME280_ERR_NOT_FOUND;
        return _lastError;
    }

    // 2. Check chip ID — must be 0x60 for BME280 (datasheet §5.4.1)
    uint8_t chipId = _readReg(BME280_REG_ID);
    if (chipId != BME280_CHIP_ID) {
        _lastError = BME280_ERR_WRONG_ID;
        return _lastError;
    }

    // 3. Soft reset to ensure a clean state
    reset();
    delay(10);  // wait for POR to complete (startup time < 2 ms per datasheet §1.1)

    // 4. Read calibration data from NVM (done once, stored in RAM)
    _readCalibration();

    // 5. Configure oversampling and filter
    //    Target: "weather monitoring" mode (datasheet §3.5.1)
    //    - Humidity   osrs ×1
    //    - Temp       osrs ×1
    //    - Pressure   osrs ×1
    //    - IIR filter off
    //    - Mode:      forced (triggered on each read() call)

    // ctrl_hum must be written BEFORE ctrl_meas for changes to take effect
    // (datasheet §5.4.3 note)
    _writeReg(BME280_REG_CTRL_HUM, BME280_OSRS_1);

    // config register: filter off, no SPI 3-wire
    // t_sb is irrelevant in forced mode but we set it cleanly
    uint8_t config = (BME280_STANDBY_0_5MS << 5) | (BME280_FILTER_OFF << 2);
    _writeReg(BME280_REG_CONFIG, config);

    // ctrl_meas: set oversampling but leave in SLEEP mode for now
    // Forced mode is triggered on each read() call
    uint8_t ctrlMeas = (BME280_OSRS_1 << 5) | (BME280_OSRS_1 << 2) | BME280_MODE_SLEEP;
    _writeReg(BME280_REG_CTRL_MEAS, ctrlMeas);

    _ready = true;
    _lastError = BME280_OK;
    return BME280_OK;
}


BME280Data BME280Driver::read() {
    BME280Data result;
    result.temperature_C = NAN;
    result.humidity_pct  = NAN;
    result.pressure_hPa  = NAN;
    result.valid         = false;

    if (!_ready) {
        _lastError = BME280_ERR_NOT_FOUND;
        return result;
    }

    // Trigger forced mode measurement:
    // Write ctrl_meas with MODE_FORCED — sensor will measure then return to sleep
    // IMPORTANT: ctrl_hum must be written before each ctrl_meas write in forced mode
    // (datasheet §5.4.3 — changes to ctrl_hum only become effective after ctrl_meas write)
    _writeReg(BME280_REG_CTRL_HUM, BME280_OSRS_1);

    uint8_t ctrlMeas = (BME280_OSRS_1 << 5) | (BME280_OSRS_1 << 2) | BME280_MODE_FORCED;
    _writeReg(BME280_REG_CTRL_MEAS, ctrlMeas);

    // Wait for measurement to complete
    // Typical measurement time with osrs ×1 all: ~8.22 ms (datasheet §9.1 formula)
    // We poll the status register rather than using a fixed delay
    if (!_waitMeasurementReady(100)) {
        _lastError = BME280_ERR_TIMEOUT;
        return result;
    }

    // Burst read all data registers: 0xF7..0xFE (8 bytes)
    // press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb, hum_msb, hum_lsb
    uint8_t data[8];
    if (!_readRegs(BME280_REG_PRESS_MSB, data, 8)) {
        _lastError = BME280_ERR_I2C;
        return result;
    }

    // Reconstruct raw 20-bit ADC values (datasheet §4, §5.4.7/8/9)
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
    int32_t adc_H = ((int32_t)data[6] << 8)  |  (int32_t)data[7];

    // Compensate — temperature must be computed first to populate _t_fine
    result.temperature_C = _compensateTemperature(adc_T);
    result.pressure_hPa  = _compensatePressure(adc_P);
    result.humidity_pct  = _compensateHumidity(adc_H);
    result.valid         = true;

    _lastError = BME280_OK;
    return result;
}


void BME280Driver::reset() {
    _writeReg(BME280_REG_RESET, BME280_RESET_VALUE);
}


// ─── Private: I²C helpers ────────────────────────────────────────────────────

bool BME280Driver::_writeReg(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission() == 0);
}

bool BME280Driver::_readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;  // repeated start

    uint8_t received = _wire->requestFrom((uint8_t)_address, len);
    if (received != len) return false;

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _wire->read();
    }
    return true;
}

uint8_t BME280Driver::_readReg(uint8_t reg) {
    uint8_t val = 0;
    _readRegs(reg, &val, 1);
    return val;
}


// ─── Private: calibration data ───────────────────────────────────────────────

void BME280Driver::_readCalibration() {
    // Read 0x88..0xA1 (26 bytes): T1..T3, P1..P9  (datasheet Table 16)
    uint8_t calib[26];
    _readRegs(BME280_REG_CALIB_00, calib, 26);

    _dig_T1 = (uint16_t)(calib[1] << 8) | calib[0];
    _dig_T2 = (int16_t) (calib[3] << 8) | calib[2];
    _dig_T3 = (int16_t) (calib[5] << 8) | calib[4];

    _dig_P1 = (uint16_t)(calib[7]  << 8) | calib[6];
    _dig_P2 = (int16_t) (calib[9]  << 8) | calib[8];
    _dig_P3 = (int16_t) (calib[11] << 8) | calib[10];
    _dig_P4 = (int16_t) (calib[13] << 8) | calib[12];
    _dig_P5 = (int16_t) (calib[15] << 8) | calib[14];
    _dig_P6 = (int16_t) (calib[17] << 8) | calib[16];
    _dig_P7 = (int16_t) (calib[19] << 8) | calib[18];
    _dig_P8 = (int16_t) (calib[21] << 8) | calib[20];
    _dig_P9 = (int16_t) (calib[23] << 8) | calib[22];

    _dig_H1 = calib[25];  // 0xA1

    // Read 0xE1..0xE7 (7 bytes): H2..H6  (datasheet Table 16)
    uint8_t hcalib[7];
    _readRegs(BME280_REG_CALIB_26, hcalib, 7);

    _dig_H2 = (int16_t)(hcalib[1] << 8) | hcalib[0];
    _dig_H3 = hcalib[2];

    // H4 and H5 have a peculiar bit layout (datasheet Table 16)
    _dig_H4 = (int16_t)((hcalib[3] << 4) | (hcalib[4] & 0x0F));
    _dig_H5 = (int16_t)((hcalib[5] << 4) | (hcalib[4] >> 4));
    _dig_H6 = (int8_t)hcalib[6];
}


// Private: wait for measurement  

bool BME280Driver::_waitMeasurementReady(uint32_t timeoutMs) {
    uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        // status register bit 3: measuring[0] — 1 while conversion running
        uint8_t status = _readReg(BME280_REG_STATUS);
        if ((status & 0x08) == 0) return true;  // measurement done
        delay(2);
    }
    return false;  // timeout
}


// Private: compensation formulas  
// Source: BME280 datasheet Rev 1.6, §4.2.3
// Using 32-bit integer path for temperature and humidity,
// 64-bit integer path for pressure (required for full accuracy).

float BME280Driver::_compensateTemperature(int32_t adc_T) {
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) *
              ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) *
            ((int32_t)_dig_T3)) >> 14;

    _t_fine = var1 + var2;

    int32_t T = (_t_fine * 5 + 128) >> 8;
    return (float)T / 100.0f;  // resolution 0.01 °C
}

float BME280Driver::_compensateHumidity(int32_t adc_H) {
    int32_t v_x1_u32r;

    v_x1_u32r = (_t_fine - (int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - ((int32_t)_dig_H4 << 20) - ((int32_t)_dig_H5 * v_x1_u32r)) +
                   (int32_t)16384) >> 15) *
                 (((((((v_x1_u32r * (int32_t)_dig_H6) >> 10) *
                      (((v_x1_u32r * (int32_t)_dig_H3) >> 11) + (int32_t)32768)) >> 10) +
                    (int32_t)2097152) * (int32_t)_dig_H2 + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                (int32_t)_dig_H1) >> 4));

    // Clamp to valid range
    if (v_x1_u32r < 0)         v_x1_u32r = 0;
    if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;

    return (float)(v_x1_u32r >> 12) / 1024.0f;  // %RH
}

float BME280Driver::_compensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;

    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_dig_P6;
    var2 = var2 + ((var1 * (int64_t)_dig_P5) << 17);
    var2 = var2 + (((int64_t)_dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_dig_P3) >> 8) + ((var1 * (int64_t)_dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_dig_P1) >> 33;

    if (var1 == 0) return 0.0f;  // avoid division by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_dig_P7) << 4);

    return (float)(uint32_t)p / 25600.0f;  // Pa → hPa  (÷256 then ÷100)
}