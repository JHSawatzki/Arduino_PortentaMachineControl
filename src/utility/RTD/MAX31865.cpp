#include "MAX31865.h"

const float MAX31865Class::Z1;
const float MAX31865Class::Z2;
const float MAX31865Class::Z4;

MAX31865Class::MAX31865Class(PinName cs, SPIClass& spi) : _cs(cs), _spi(&spi), _spi_settings(1000000, MSBFIRST, SPI_MODE1) {
}

bool MAX31865Class::begin() {
    if(!_begun) {
        _spi->begin();

        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);

        setRTDBias(false);
        setRTDAutoConvert(false);
        setRTDThresholds(0, 0xFFFF);
        clearRTDFault();
        _async_state = IDLE; // added for asynchronous

        _begun = true;
    }

    return true;
}

void MAX31865Class::end() {
    if(_begun) {
        pinMode(_cs, INPUT);
        digitalWrite(_cs, LOW);
        _spi->end();
        _begun = false;
    }
}

void MAX31865Class::setRTDThresholds(uint16_t lowerThreshold, uint16_t upperThreshold) {
    writeByte(MAX31865_L_FAULT_LSB_REG, lowerThreshold & 0xFF);
    writeByte(MAX31865_L_FAULT_MSB_REG, lowerThreshold >> 8);
    writeByte(MAX31865_H_FAULT_LSB_REG, upperThreshold & 0xFF);
    writeByte(MAX31865_H_FAULT_MSB_REG, upperThreshold >> 8);
}

uint16_t MAX31865Class::getRTDLowerThreshold() {
    return readWord(MAX31865_L_FAULT_MSB_REG);
}

uint16_t MAX31865Class::getRTDUpperThreshold() {
    return readWord(MAX31865_H_FAULT_MSB_REG);
}

void MAX31865Class::setRTDType(temperature_probe_t probeType) {
    // sets 2 or 4 wire
    if (probeType == PROBE_RTD_PT100_3W) {
        writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) | MAX31865_CONFIG_3WIRE);
    } else {
        writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) & ~MAX31865_CONFIG_3WIRE);
    }
    _current_probe_type = probeType;
}

temperature_probe_t MAX31865Class::getRTDType() {
    return _current_probe_type;
}

void MAX31865Class::setRTDAutoConvert(bool enabled) {
    uint8_t config = readByte(MAX31865_CONFIG_REG);
    if (enabled) {
        config |= MAX31865_CONFIG_CONV_MODE_AUTO; // enable continuous conversion
    } else {
        config &= ~MAX31865_CONFIG_CONV_MODE_AUTO; // disable continuous conversion
    }
    writeByte(MAX31865_CONFIG_REG, config);
    if (enabled && !_continuous_mode_enabled) {
        if (_50hz_filter_enabled) {
            delay(70);
        } else {
            delay(60);
        }
    }
    _continuous_mode_enabled = enabled;
}

void MAX31865Class::setRTD50HzFilter(bool enabled) {
    uint8_t config = readByte(MAX31865_CONFIG_REG);
    if (enabled) {
        config |= MAX31865_CONFIG_FILTER_50HZ;
    } else {
        config &= ~MAX31865_CONFIG_FILTER_50HZ;
    }
    writeByte(MAX31865_CONFIG_REG, config);
    _50hz_filter_enabled = enabled;
}

void MAX31865Class::setRTDBias(bool enabled) {
    uint8_t config = readByte(MAX31865_CONFIG_REG);
    if (enabled) {
        config |= MAX31865_CONFIG_BIAS; // enable bias
    } else {
        config &= ~MAX31865_CONFIG_BIAS; // disable bias
    }
    writeByte(MAX31865_CONFIG_REG, config);
    _bias_voltage_enabled = enabled;
}

void MAX31865Class::clearRTDFault() {
    writeByte(MAX31865_CONFIG_REG, (readByte(MAX31865_CONFIG_REG) & ~0x2C) | MAX31865_CONFIG_FAULT_STAT);
}

uint8_t MAX31865Class::readRTDFault(max31865_fault_cycle_t faultCycle) {
    if (faultCycle) {
        uint8_t cfgRegVal = readByte(MAX31865_CONFIG_REG);
        cfgRegVal &= 0x11; // mask out wire and filter bits
        switch (faultCycle) {
            case MAX31865_FAULT_AUTO:
                writeByte(MAX31865_CONFIG_REG, (cfgRegVal | 0b10000100));
                delay(1);
                break;
            case MAX31865_FAULT_MANUAL_RUN:
                writeByte(MAX31865_CONFIG_REG, (cfgRegVal | 0b10001000));
                return 0;
            case MAX31865_FAULT_MANUAL_FINISH:
                writeByte(MAX31865_CONFIG_REG, (cfgRegVal | 0b10001100));
                return 0;
            case MAX31865_FAULT_NONE:
            default:
                break;
        }
    }
    return readByte(MAX31865_FAULT_STAT_REG);
}

uint16_t MAX31865Class::readRTD() {
    clearRTDFault();

    if (!_continuous_mode_enabled) {
        if (!_bias_voltage_enabled) {
            // enable bias
            writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) | MAX31865_CONFIG_BIAS);
            delay(10);
        }
        writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) | MAX31865_CONFIG_CONV_MODE_ONE_SHOT);
        if (_50hz_filter_enabled) {
            delay(75);
        } else {
            delay(65);
        }
    }

    uint16_t rtdValueRaw = readWord(MAX31865_RTD_MSB_REG);

    if (!_bias_voltage_enabled) {
        // disable bias
        writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) & ~MAX31865_CONFIG_BIAS);
    }

    // remove fault
    rtdValueRaw >>= 1;

    return rtdValueRaw;
}

bool MAX31865Class::readRTDAsync(uint16_t& rtdValueRaw) {
    bool valueAvailable = false;

    switch (_async_state) {
        case IDLE: // Idle
            clearRTDFault();
            if (!_bias_voltage_enabled) {
                // enable bias
                writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) | MAX31865_CONFIG_BIAS);
            }
            _async_timer = millis();
            _async_state = SETTLING;
            break;

        case SETTLING: // Bias voltage enabled, waiting to settle
            if (millis() - _async_timer >= 10) {
                writeByte(MAX31865_CONFIG_REG, readByte(MAX31865_CONFIG_REG) | MAX31865_CONFIG_CONV_MODE_ONE_SHOT);
                _async_timer = millis();
                _async_state = CONVERTING;
            }
            break;

        case CONVERTING: // Conversion started, waiting for completion
            if (millis() - _async_timer >= (_50hz_filter_enabled ? 75 : 65)) {
                rtdValueRaw = readWord(MAX31865_RTD_MSB_REG);
                rtdValueRaw >>= 1; // remove fault
                _async_state = IDLE; // get ready for next time
                valueAvailable = true; // signal computation is done
                //TODO disable bias??
            }
            break;
    }
    return valueAvailable;
}

float MAX31865Class::readRTDResistance(float refResistanceValue) {
    // read RTD raw value and calculate resistance value
    return calculateRTDResistance(readRTD(), refResistanceValue);
}

float MAX31865Class::readRTDTemperature(float rtdNominalValue, float refResistanceValue) {
    return calculateRTDTemperature(readRTD(), rtdNominalValue, refResistanceValue);
}

float MAX31865Class::calculateRTDResistance(uint16_t rtdValueRaw, float refResistanceValue) {
    float rtdResistanceValue = rtdValueRaw;
    rtdResistanceValue /= 32768;
    rtdResistanceValue *= refResistanceValue;
    return rtdResistanceValue;
}

float MAX31865Class::calculateRTDTemperature(float rtdResistanceValue, float rtdNominalValue) {
    float Z3 = (4 * RTD_B) / rtdNominalValue;

    float temp = Z2 + (Z3 * rtdResistanceValue);
    temp = (sqrt(temp) + Z1) / Z4;

    if (temp >= 0)
        return temp;
    
    // ugh.
    rtdResistanceValue /= rtdNominalValue;
    rtdResistanceValue *= 100; // normalize to 100 ohm

    float rpoly = rtdResistanceValue;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= rtdResistanceValue; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= rtdResistanceValue; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= rtdResistanceValue; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= rtdResistanceValue; // ^5
    temp += 1.5243e-10 * rpoly;

    return temp;
}

float MAX31865Class::calculateRTDTemperature(uint16_t rtdValueRaw, float rtdNominalValue, float refResistanceValue) {
    return calculateRTDTemperature(calculateRTDResistance(rtdValueRaw, refResistanceValue), rtdNominalValue);
}

uint8_t MAX31865Class::readByte(uint8_t addr) {
    addr &= 0x7F;
    uint8_t read = 0;

    digitalWrite(_cs, LOW);

    _spi->beginTransaction(_spi_settings);
    _spi->transfer(addr);
    _spi->transfer(&read, 1);
    _spi->endTransaction();

    digitalWrite(_cs, HIGH);

    return read;
}

uint16_t MAX31865Class::readWord(uint8_t addr) {
    uint8_t buffer[2] = {0, 0};
    addr &= 0x7F; // make sure top bit is not set

    digitalWrite(_cs, LOW);

    _spi->beginTransaction(_spi_settings);
    _spi->transfer(addr);
    buffer[0] = _spi->transfer(0x00);
    buffer[1] = _spi->transfer(0x00);
    _spi->endTransaction();

    digitalWrite(_cs, HIGH);

    uint16_t read = buffer[0];
    read <<= 8;
    read |= buffer[1];

    return read;
}

void MAX31865Class::writeByte(uint8_t addr, uint8_t data) {
    addr |= 0x80; // make sure top bit is set
    uint8_t buffer[2] = {addr, data};

    digitalWrite(_cs, LOW);

    _spi->beginTransaction(_spi_settings);
    _spi->transfer(buffer,2);
    _spi->endTransaction();

    digitalWrite(_cs, HIGH);
}
