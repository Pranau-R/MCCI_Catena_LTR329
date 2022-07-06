/*

Module: MCCI_Catena_LTR329.h

Function:
        Implementation for MCCI Catena LTR329 sensor Library.

Copyright and License:
        See accompanying LICENSE file.

Author:
        Pranau, MCCI Corporation   June 2022

*/

#include "MCCI_Catena_LTR329.h"

using namespace McciCatenaLtr329;

cLTR329::cLTR329()
    {
    }

cLTR329::~cLTR329()
    {
    }

bool cLTR329::begin()
    {
    // Initial wait after Power Up
    delay(100);
    // Initialize I2C
    if (this->m_wire == nullptr)
        {
        return this->setLastError(Error::NoWire);
        }
    if (this->isRunning())
        {
        return true;
        }

    this->m_wire->begin();
    // assume it's in idle state.
    this->m_state = this->m_state == State::End ? State::Triggered : State::Initial;
    // Initialize pFactor
    float pFactor = LTR329_PFACTOR;

    // Reset LTR329
    reset();
    // Initialize Parameters
    m_isActiveMode = true;
    m_gain = LTR329_ALS_GAIN_x1;
    // Set Control REG
    writetControl(m_isActiveMode, m_gain);
    // Wait for Activate LTR329
    delay(10);
    }

void cLTR329::reset()
    {
    #ifdef DEBUG
        Serial.print("[LTR329] Resetting");
    #endif

    ALS_CONTR_REG ctrl = {};
    ctrl.resetState = true;
    writeByte(LTR329_ADDR_ALS_CONTROL, ctrl.raw);

    // Wait for resetting
    do
        {
        Serial.print(".");
        ctrl.raw = readByte(LTR329_ADDR_ALS_CONTROL);
        }
    while(ctrl.resetState);

    #ifdef DEBUG
        Serial.println("Finished");
    #endif
    }

/*

Name:   cLTR329::readLux()

Function:
        Read a lux data from LTR329 sensor.

Definition:
        float cLTR329::readLux(
                void)

Description:
        It is used to read a Lux from a light source. The light data is read
        and it's ratio is been calculated. Using the ratio and PFACTOR, few calculation and condition
        are made to find the final Lux value.

Returns:
        This function returns Lux in floating integer.

*/

float cLTR329::readLux()
    {
    ALS_PS_STATUS_REG status;

    do
        {
        status = readStatus();
        #ifdef DEBUG
            Serial.print("LTR329 STATUS REG: 0b");
            Serial.println(status.raw, BIN);
        #endif
        }

    while(!status.isNewData || status.isInValid);

    uint16_t data_ch0;
    uint16_t data_ch1;

    data_ch1 = readAlsData(1);
    data_ch0 = readAlsData(0);

    #ifdef DEBUG
        Serial.print("Read Data CH1: ");
        Serial.println(data_ch1);
        Serial.print("Read Data CH0: ");
        Serial.println(data_ch0);
    #endif

    if(data_ch0 + data_ch1 == 0)
        {
        return 0;
        }

    float ratio = data_ch1 / (data_ch0 + data_ch1);
    float lux;
    float pFactor;
    Serial.print ("ALS_GAIN[m_gain] is : ");
    Serial.println (ALS_GAIN[m_gain]);

    Serial.print ("ALS_INT[m_intTime] is : ");
    Serial.println (ALS_INT[m_intTime]);

    Serial.print ("pFactor is : ");
    Serial.println (pFactor);

    float scale = 1/(ALS_GAIN[m_gain] * ALS_INT[m_intTime] * pFactor);

    struct luxConstant
        {
        float ch0scale, ch1scale;
        };

    struct luxConstant param;

    if(ratio < 0.45)
        {
        const struct luxConstant param = {1.7743, 1.1059};
        }
    else if(ratio < 0.64 && ratio >= 0.45)
        {
        const struct luxConstant param = {4.2785, -1.9548};
        }
    else if(ratio < 0.85 && ratio >= 0.64)
        {
        const struct luxConstant param = {0.5926, 0.1185};
        }
    else
        {
        const struct luxConstant param = {0, 0};
        }

    lux = (data_ch0 * param.ch0scale + data_ch1 * param.ch1scale) * scale;
    return lux;
    }

void cLTR329::writetControl(bool isActiveMode, ALS_GAIN_Enum gain)
    {
    m_isActiveMode = isActiveMode;
    m_gain = gain;

    ALS_CONTR_REG ctrl = {};
    ctrl.activeMode = m_isActiveMode;
    ctrl.gain = m_gain;

    writeByte(LTR329_ADDR_ALS_CONTROL, ctrl.raw);
    }

ALS_CONTR_REG cLTR329::readControl()
    {
    ALS_CONTR_REG ctrl;
    ctrl.raw = readByte(LTR329_ADDR_ALS_CONTROL);
    return ctrl;
    }

void cLTR329::writeMeasRate(ALS_INT_Enum intTime, ALS_MEAS_Enum measRate)
    {
    m_intTime = intTime;
    m_measRate = measRate;

    ALS_MEAS_RATE_REG mr = {};
    mr.intTime = m_intTime;
    mr.measRate = m_measRate;

    writeByte(LTR329_ADDR_ALS_MEAS_RATE, mr.raw);
    }

ALS_MEAS_RATE_REG cLTR329::readMeasRate()
    {
    ALS_MEAS_RATE_REG mr;
    mr.raw = readByte(LTR329_ADDR_ALS_MEAS_RATE);
    return mr;
    }

ALS_PS_STATUS_REG cLTR329::readStatus()
    {
    ALS_PS_STATUS_REG status;
    status.raw = readByte(LTR329_ADDR_ALS_STATUS);
    return status;
    }

uint8_t cLTR329::readManufacturerId()
    {
    uint8_t data = readByte(LTR329_ADDR_MANUFAC_ID);
    return data;
    }

uint8_t cLTR329::readPartNumber()
    {
    uint8_t data = readByte(LTR329_ADDR_PART_ID);
    return data >> 4;
    }

uint8_t cLTR329::readRevisionId()
    {
    uint8_t data = readByte(LTR329_ADDR_PART_ID);
    return data & 0x0F;
    }

uint8_t cLTR329::readByte(uint8_t addr)
    {
    uint8_t rdData;
    uint8_t rdDataCount;
    do
        {
        Wire.beginTransmission(LTR329_I2C_ADDRESS);
        Wire.write(addr);
        Wire.endTransmission(false); // Restart
        delay(10);
        rdDataCount = Wire.requestFrom(LTR329_I2C_ADDRESS, 1);
        }

    while(rdDataCount == 0);

    while(Wire.available())
        {
        rdData = Wire.read();
        }

    return rdData;
    }

uint16_t cLTR329::readAlsData(uint8_t ch)
    {
    uint8_t data_lsb, data_msb;
    uint8_t addr_lsb, addr_msb;

    if(ch == 0)
        {
        addr_lsb = LTR329_ADDR_ALS_DATA_CH_0_0;
        addr_msb = LTR329_ADDR_ALS_DATA_CH_0_1;
        }
    else
        {
        addr_lsb = LTR329_ADDR_ALS_DATA_CH_1_0;
        addr_msb = LTR329_ADDR_ALS_DATA_CH_1_1;
        }

    data_lsb = readByte(addr_lsb);
    data_msb = readByte(addr_msb);

    #ifdef DEBUG
        char tempString[64];
        sprintf(tempString, "Read CH %d LSB: %d", ch, data_lsb);
        Serial.println(tempString);
        sprintf(tempString, "Read CH %d MSB: %d", ch, data_msb);
        Serial.println(tempString);
    #endif

    return ((uint16_t)data_msb << 8) | (uint16_t)data_lsb;
    }

void cLTR329::writeByte(uint8_t addr, uint8_t data)
    {
    Wire.beginTransmission(LTR329_I2C_ADDRESS);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission();
    }