/**
 * @file TCTempProbeClass.cpp
 * @author Jan Henrik Sawatzki
 * @brief Source file for the shared Resistance Temperature Detector (RTD) and Thermocouple (TC) temperature sensor connectors of the Portenta Machine Control.
 */

/* Includes -----------------------------------------------------------------*/
#include "TempProbeClass.h"

#if __has_include("portenta_info.h")
#include "portenta_info.h"
#define TRY_REV2_RECOGNITION
uint8_t* boardInfo();
#define PMC_R2_SKU  (24 << 8 | 3)
#endif

/* Functions -----------------------------------------------------------------*/
TempProbeClass::TempProbeClass(PinName ch_sel0_pin,
                                PinName ch_sel1_pin,
                                PinName ch_sel2_pin,
                                PinName tc_cs_pin,
                                PinName rtd_cs_pin,
                                PinName rtd_th_pin)
: MAX31855Class(tc_cs_pin), MAX31865Class(rtd_cs_pin), _ch_sel0{ch_sel0_pin}, _ch_sel1{ch_sel1_pin}, _ch_sel2{ch_sel2_pin}, _rtd_th{rtd_th_pin} {
}

TempProbeClass::~TempProbeClass() {
}

bool TempProbeClass::beginTC() {
    bool result = true;
    if(!_tc_init) {
        if(!_rtd_init) {
            initDO();
        }

        result = MAX31855Class::begin();

        _tc_init = true;
    }

    return result;
}

bool TempProbeClass::beginRTD() {
    bool result = true;
    if(!_rtd_init) {
        if(!_tc_init) {
            initDO();
        }

        result = MAX31865Class::begin();

        _rtd_init = true;
    }

    return result;
}

bool TempProbeClass::begin() {
    bool result;

    result = beginTC();
    if (result) {
        result = beginRTD();
    }

    return result;
}

void TempProbeClass::endTC() {
    if(_tc_init) {
        MAX31855Class::end();

        _tc_init = false;

        if(!_rtd_init) {
            deinitDO();
        }
    }
}

void TempProbeClass::endRTD() {
    if(_rtd_init) {
        MAX31865Class::end();

        _rtd_init = false;

        if(!_tc_init) {
            deinitDO();
        }
    }
}

void TempProbeClass::end() {
    endTC();
    endRTD();
}

void TempProbeClass::selectChannel(uint8_t channel, temperature_probe_t probeType) {
    if (channel >= 0 && channel <= 2) {
        channel = channelReverse(channel);

        switchProbe(channel, probeType);

        if (switchChannel(channel)) {
            setupSwitchDelay(probeType);
            delay(_switch_delay);
        }
    }
}

bool TempProbeClass::selectChannelAsync(uint8_t channel, temperature_probe_t probeType) {
    if (channel >= 0 && channel <= 2) {
        channel = channelReverse(channel);

        switchProbe(channel, probeType);

        if (switchChannel(channel)) {
            _async_timer = millis();
            _async_state = SWITCHING;
            setupSwitchDelay(probeType);
        } else {
            switch (_async_state) {
                case SWITCHING: // Channel switching
                    if (millis() - _async_timer >= _switch_delay) {
                        _async_state = SELECTED;
                    }
                    break;
                case SELECTED: // Channel selected
                    break;
            }
        }
    }

    return (_async_state == SELECTED) ? true : false;
}

void TempProbeClass::initDO() {
    pinMode(_ch_sel0, OUTPUT);
    pinMode(_ch_sel1, OUTPUT);
    pinMode(_ch_sel2, OUTPUT);

    pinMode(_rtd_th, OUTPUT);

    digitalWrite(_ch_sel0, LOW);
    digitalWrite(_ch_sel1, LOW);
    digitalWrite(_ch_sel2, LOW);

    digitalWrite(_rtd_th, LOW);
}

void TempProbeClass::deinitDO() {
    pinMode(_ch_sel0, INPUT);
    pinMode(_ch_sel1, INPUT);
    pinMode(_ch_sel2, INPUT);

    pinMode(_rtd_th, INPUT);

    digitalWrite(_ch_sel0, LOW);
    digitalWrite(_ch_sel1, LOW);
    digitalWrite(_ch_sel2, LOW);

    digitalWrite(_rtd_th, LOW);
}

uint8_t TempProbeClass::channelReverse(uint8_t channel) {
#ifdef TRY_REV2_RECOGNITION
    // check if OTP data is present AND the board is mounted on a r2 carrier
    auto info = (PortentaBoardInfo*)boardInfo();
    if (info->magic == 0xB5 && info->carrier == PMC_R2_SKU) {
        // reverse channels 0 and 2
        switch (channel) {
            case 0:
                channel = 2;
                break;
            case 2:
                channel = 0;
                break;
            default:
                break;
        }
    }
#endif
#undef TRY_REV2_RECOGNITION
    return channel;
}

bool TempProbeClass::switchChannel(uint8_t channel) {
    if (_current_channel != channel) {
        switch(channel) {
            case 0:
                digitalWrite(_ch_sel0, HIGH);
                digitalWrite(_ch_sel1, LOW);
                digitalWrite(_ch_sel2, LOW);
                break;
            case 1:
                digitalWrite(_ch_sel0, LOW);
                digitalWrite(_ch_sel1, HIGH);
                digitalWrite(_ch_sel2, LOW);
                break;
            case 2:
                digitalWrite(_ch_sel0, LOW);
                digitalWrite(_ch_sel1, LOW);
                digitalWrite(_ch_sel2, HIGH);
                break;
            default:
                digitalWrite(_ch_sel0, LOW);
                digitalWrite(_ch_sel1, LOW);
                digitalWrite(_ch_sel2, LOW);
                break;
        }
        _current_channel = channel;
        return true;
    } else {
        return false;
    }
}

void TempProbeClass::switchProbe(uint8_t channel, temperature_probe_t probeType) {
    switch (probeType) {
        case PROBE_TC_K:
        case PROBE_TC_J:
        case PROBE_TC_T:
            digitalWrite(_rtd_th, LOW);
            if (_current_channel != channel || _current_probe_type[channel] != probeType) {
                MAX31855Class::setTCType(probeType);
            }
            break;

        case PROBE_RTD_PT100_2W:
        case PROBE_RTD_PT100_3W:
            digitalWrite(_rtd_th, HIGH);
            if (_current_channel != channel || _current_probe_type[channel] != probeType) {
                MAX31865Class::setRTDType(probeType);
            }
            break;
    }
    _current_probe_type[channel] = probeType;
}

void TempProbeClass::setupSwitchDelay(temperature_probe_t probeType) {
    switch (probeType) {
        case PROBE_TC_K:
        case PROBE_TC_J:
        case PROBE_TC_T:
            _switch_delay = 150;
            break;

        case PROBE_RTD_PT100_2W:
        case PROBE_RTD_PT100_3W:
            _switch_delay = 75;
            break;

        default:
            _switch_delay = 150;
            break;
    }
}

TempProbeClass MachineControl_TempProbe;
/**** END OF FILE ****/
