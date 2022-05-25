/*

Module: MCCI_Catena_LTR329.h

Function:
    Implementation for MCCI Catena LTR329 sensor Library.

Copyright and License:
    This file copyright (C) 2020 by

        MCCI Corporation
        3520 Krums Corners Road
        Ithaca, NY  14850

    See accompanying LICENSE file for copyright and license information..

Author:
    Pranau, MCCI Corporation   May 2022

*/

#include "MCCI_Catena_LTR329.h"
#include <stdint.h>
#include <stdbool.h>

using namespace McciCatenaLtr329;


cLTR329::cLTR329() {
}
#if defined(ESP32) || defined(ESP8266)
cLTR329::cLTR329(uint8_t sda, uint8_t scl) 
: sda(sda), scl(scl) {
}
#endif
cLTR329::~cLTR329() {

}

void cLTR329::begin() {
    // Initial wait after Power Up
    delay(100);
    // Initialize I2C
    if(sda != -1 && scl != -1) {
        Wire.begin(sda, scl); 
    }
    else {
        Wire.begin(); 
    }
    // Reset LTR329
    reset(); 
    // Initialize Parameters
    isActiveMode = true; 
    gain = cLTR329_ALS_GAIN_x1; 
    // Set Control REG
    writetControl(isActiveMode, gain); 
    // Wait for Activate LTR329
    delay(10); 
}

void cLTR329::reset() {
#ifdef DEBUG
    Serial.print("[LTR329] Resetting"); 
#endif
    ALS_CONTR_REG ctrl = {}; 
    ctrl.resetState = true; 
    writeByte(cLTR329_ADDR_ALS_CONTROL, ctrl.raw); 

    // Wait for resetting
    do {
        Serial.print("."); 
        ctrl.raw = readByte(cLTR329_ADDR_ALS_CONTROL); 
    } while(ctrl.resetState); 
#ifdef DEBUG
    Serial.println("Finished"); 
#endif
}

float cLTR329::readLux() {
    ALS_PS_STATUS_REG status; 
    do {
        status = readStatus(); 
#ifdef DEBUG
        Serial.print("LTR329 STATUS REG: 0b"); 
        Serial.println(status.raw, BIN); 
#endif
    } while(!status.isNewData || status.isInValid); 

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

    if(data_ch0 + data_ch1 == 0) return 0; // Avoid to Div by Zero
    float ratio = data_ch1 / (data_ch0 + data_ch1); 
    float lux; 
    float pFactor = cLTR329_PFACTOR; 
    
    if(ratio < 0.45) {
        lux = (1.7743 * data_ch0 + 1.1059 * data_ch1) / ALS_GAIN[gain] / ALS_INT[intTime] / pFactor; 
    }
    else if(ratio < 0.64 &&  ratio >= 0.45) {
        lux = (4.2785 * data_ch0 - 1.9548 * data_ch1) / ALS_GAIN[gain] / ALS_INT[intTime] / pFactor; 
    }
    else if(ratio < 0.85 &&  ratio >= 0.64) {
        lux = (0.5926 * data_ch0 + 0.1185 * data_ch1) / ALS_GAIN[gain] / ALS_INT[intTime] / pFactor; 
    }
    else {
        lux = 0; 
    }
    
    return lux; 
}

void cLTR329::writetControl(bool _isActiveMode, ALS_GAIN_Enum _gain) {
    isActiveMode = _isActiveMode; 
    gain = _gain; 

    ALS_CONTR_REG ctrl = {}; 
    ctrl.activeMode = isActiveMode; 
    ctrl.gain = gain; 

    writeByte(cLTR329_ADDR_ALS_CONTROL, ctrl.raw); 
}

ALS_CONTR_REG cLTR329::readControl() {
    ALS_CONTR_REG ctrl; 
    ctrl.raw = readByte(cLTR329_ADDR_ALS_CONTROL); 
    return ctrl; 
}

void cLTR329::writeMeasRate(ALS_INT_Enum _intTime, ALS_MEAS_Enum _measRate) {
    intTime = _intTime; 
    measRate = _measRate; 

    ALS_MEAS_RATE_REG mr = {}; 
    mr.intTime = intTime; 
    mr.measRate = measRate; 

    writeByte(cLTR329_ADDR_ALS_MEAS_RATE, mr.raw); 
}

ALS_MEAS_RATE_REG cLTR329::readMeasRate() {
    ALS_MEAS_RATE_REG mr; 
    mr.raw = readByte(cLTR329_ADDR_ALS_MEAS_RATE); 
    return mr; 
}

ALS_PS_STATUS_REG cLTR329::readStatus() {
    ALS_PS_STATUS_REG status; 
    status.raw = readByte(cLTR329_ADDR_ALS_STATUS); 
    return status; 
}


uint8_t cLTR329::readManufacturerId() {
    uint8_t data = readByte(cLTR329_ADDR_MANUFAC_ID); 
    return data; 
}

uint8_t cLTR329::readPartNumber() {
    uint8_t data = readByte(cLTR329_ADDR_PART_ID); 
    return data >> 4; 
}

uint8_t cLTR329::readRevisionId() {
    uint8_t data = readByte(cLTR329_ADDR_PART_ID); 
    return data & 0x0F; 
}

uint8_t cLTR329::readByte(uint8_t addr) {
    uint8_t rdData; 
    uint8_t rdDataCount; 
    do {
        Wire.beginTransmission(cLTR329_I2C_ADDRESS); 
        Wire.write(addr); 
        Wire.endTransmission(false); // Restart
        delay(10); 
        rdDataCount = Wire.requestFrom(cLTR329_I2C_ADDRESS, 1); 
    } while(rdDataCount == 0); 
    while(Wire.available()) {
        rdData = Wire.read(); 
    }
    return rdData; 

}

uint16_t cLTR329::readAlsData(uint8_t ch) {
    uint8_t data_lsb, data_msb; 
    uint8_t addr_lsb, addr_msb; 
    if(ch == 0) {
        addr_lsb = cLTR329_ADDR_ALS_DATA_CH_0_0; 
        addr_msb = cLTR329_ADDR_ALS_DATA_CH_0_1; 
    }
    else {
        addr_lsb = cLTR329_ADDR_ALS_DATA_CH_1_0; 
        addr_msb = cLTR329_ADDR_ALS_DATA_CH_1_1; 
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

void cLTR329::writeByte(uint8_t addr, uint8_t data) {
    Wire.beginTransmission(cLTR329_I2C_ADDRESS); 
    Wire.write(addr); 
    Wire.write(data); 
    Wire.endTransmission(); 
}