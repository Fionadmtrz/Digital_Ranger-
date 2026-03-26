/**
 * @file    SEN0311.h
 * @brief   Driver for DFRobot SEN0311 — A02YYUW Waterproof Ultrasonic Distance Sensor
 */

#pragma once

#include <Arduino.h>

// Protocol constants 
#define SEN0311_BAUD_RATE       9600
#define SEN0311_PACKET_SIZE     4
#define SEN0311_HEADER          0xFF
#define SEN0311_MIN_DISTANCE_MM 30     // 3 cm blind zone
#define SEN0311_MAX_DISTANCE_MM 4500   // 450 cm max range
#define SEN0311_RESPONSE_TIME_MS 100   // one packet every ~100 ms
#define SEN0311_READ_TIMEOUT_MS  500   // wait up to 500 ms for a valid packet

// Error codes 
#define SEN0311_OK               0
#define SEN0311_ERR_TIMEOUT     -1   // no packet received within timeout
#define SEN0311_ERR_CHECKSUM    -2   // checksum mismatch (noise/bad packet)
#define SEN0311_ERR_BLIND_ZONE  -3   // distance < 3 cm (blind zone)
#define SEN0311_ERR_OUT_OF_RANGE -4  // distance > 450 cm (out of range)

// Data structure 

struct SEN0311Data {
    uint16_t distance_mm;   ///< Distance to water surface in mm 
    float    distance_cm;   ///< Same in cm (
    bool     valid;         ///< true if reading passed checksum and range checks
};

// Driver class 

class SEN0311 {
public:
    /**
     * @brief Construct a SEN0311 driver.
     */
    SEN0311(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin);

    /**
     * @brief Initialise the UART port at 9600 bps.
     */
    void begin();

    /**
     * @brief Read one distance measurement from the sensor.
     */
    SEN0311Data read();

    /**
     * @brief Flush the UART receive buffer.
     */
    void flush();

    /**
     * @brief Return the last error code.
     */
    int8_t lastError() const { return _lastError; }

    /**
     * @brief Return true if begin() has been called.
     */
    bool isReady() const { return _ready; }

private:
    HardwareSerial& _serial;
    uint8_t         _rxPin;
    uint8_t         _txPin;
    bool            _ready;
    int8_t          _lastError;

    // Returns true if packet is valid
    bool _parsePacket(uint8_t* buf, uint16_t& distanceOut);
};