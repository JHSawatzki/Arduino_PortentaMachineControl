#ifndef MAX31865_H
#define MAX31865_H

#include <Arduino.h>
#include <mbed.h>
#include <SPI.h>
#include "pins_mc.h"
#include "enums_mc.h"

#define MAX31865_CONFIG_REG 0x00

#define MAX31865_CONFIG_BIAS 0x80

#define MAX31865_CONFIG_CONV_MODE_AUTO 0x40
#define MAX31865_CONFIG_CONV_MODE_OFF 0x00
#define MAX31865_CONFIG_CONV_MODE_ONE_SHOT 0x20

#define MAX31865_CONFIG_3WIRE 0x10
#define MAX31865_CONFIG_2WIRE 0x00

#define MAX31865_CONFIG_FAULT_STAT 0x02

#define MAX31865_CONFIG_FILTER_50HZ 0x01
#define MAX31865_CONFIG_FILTER_60HZ 0x00

#define MAX31865_RTD_MSB_REG 0x01
#define MAX31865_RTD_LSB_REG 0x02
#define MAX31865_H_FAULT_MSB_REG 0x03
#define MAX31865_H_FAULT_LSB_REG 0x04
#define MAX31865_L_FAULT_MSB_REG 0x05
#define MAX31865_L_FAULT_LSB_REG 0x06
#define MAX31865_FAULT_STAT_REG 0x07

#define MAX31865_FAULT_HIGH_THRESH 0x80
#define MAX31865_FAULT_LOW_THRESH 0x40
#define MAX31865_FAULT_LOW_REFIN 0x20
#define MAX31865_FAULT_HIGH_REFIN 0x10
#define MAX31865_FAULT_LOW_RTDIN 0x08
#define MAX31865_FAULT_OVER_UNDER_VOLTAGE 0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

// The value of the Rref resistor.
#define RREF 400.0

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100
#define RNOMINAL 100.0

enum max31865_conversion_state_t : byte {IDLE, SETTLING, CONVERTING}; // for asynchronous mode

typedef enum {
    MAX31865_FAULT_NONE = 0,
    MAX31865_FAULT_AUTO,
    MAX31865_FAULT_MANUAL_RUN,
    MAX31865_FAULT_MANUAL_FINISH
} max31865_fault_cycle_t;

class MAX31865Class {
public:
    MAX31865Class(PinName cs = MC_RTD_CS_PIN, SPIClass& spi = SPI);

    bool begin();
    void end();

    void setRTDThresholds(uint16_t lowerThreshold, uint16_t upperThreshold);
    uint16_t getRTDLowerThreshold();
    uint16_t getRTDUpperThreshold();

    void setRTDType(temperature_probe_t probeType);
    temperature_probe_t getRTDType();

    void setRTDAutoConvert(bool enabled);
    void setRTD50HzFilter(bool enabled);
    void setRTDBias(bool enabled);

    uint8_t readRTDFault(max31865_fault_cycle_t faultCycle = MAX31865_FAULT_AUTO);
    void clearRTDFault();

    uint16_t readRTD();
    bool readRTDAsync(uint16_t& rtdValueRaw);

    float readRTDResistance(float refResistanceValue = RREF);
    float readRTDTemperature(float rtdNominalValue = RNOMINAL, float refResistanceValue = RREF);

    float calculateRTDResistance(uint16_t rtdValueRaw, float refResistanceValue = RREF);
    float calculateRTDTemperature(float rtdResistanceValue, float rtdNominalValue = RNOMINAL);
    float calculateRTDTemperature(uint16_t rtdValueRaw, float rtdNominalValue = RNOMINAL, float refResistanceValue = RREF);

private:
    PinName _cs;
    SPIClass* _spi;
    SPISettings _spi_settings;
    temperature_probe_t _current_probe_type = PROBE_NC;
    bool _begun;
    bool _continuous_mode_enabled; // continuous conversion
    bool _50hz_filter_enabled; // 50Hz filter
    bool _bias_voltage_enabled; // bias voltage
    uint32_t _async_timer; // timer for asynchronous mode, added Sylvain Boyer
    max31865_conversion_state_t _async_state; // state for asynchronous mode

    static constexpr float Z1 = -RTD_A;
    static constexpr float Z2 = RTD_A * RTD_A - (4 * RTD_B);
    static constexpr float Z4 = 2 * RTD_B;

    uint8_t readByte(uint8_t addr);
    uint16_t readWord(uint8_t addr);
    void writeByte(uint8_t addr, uint8_t data);
};

#endif
