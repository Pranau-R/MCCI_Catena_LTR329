/*

Module: ltr329_simple.ino

Function:
        Simple example for LTR329 Optical sensor.

Copyright and License:
        This file copyright (C) 2020 by

        MCCI Corporation
        3520 Krums Corners Road
        Ithaca, NY  14850

    See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   May 2022

*/

# include <MCCI_Catena_LTR329.h>
# include <stdio.h>

/****************************************************************************\
|
|   Manifest constants & typedefs.
|
\****************************************************************************/

using namespace McciCatenaLtr329;

/****************************************************************************\
|
|   Read-only data.
|
\****************************************************************************/

// I2C pins for MCCI Catena4610
//#define I2C_SCL 35
//#define I2C_SDA 36

cLTR329 gLtr329 {Wire};

/****************************************************************************\
|
|   Code.
|
\****************************************************************************/

void setup()
    {
    Serial.begin(115200);

    // Start LTR329
    gLtr329.begin();
    Serial.println("[LTR329] begin. ");

    char tempString[64] = {};
    Serial.print("");
    sprintf(tempString, "[LTR329] MANUFAC ID: 0x%02x", gLtr329.readManufacturerId());
    Serial.println(tempString);
    sprintf(tempString, "[LTR329] PART NUMBER: 0x%02x", gLtr329.readPartNumber());
    Serial.println(tempString);
    Serial.print("[LTR329] STATUS REG: 0b");
    Serial.println(gLtr329.readStatus().raw, BIN);

    gLtr329.writetControl(true, cLTR239_ALS_GAIN_x8);
    Serial.print("[LTR329] CTRL REG: 0b");
    Serial.println(gLtr329.readControl().raw, BIN);

    gLtr329.writeMeasRate(cLTR239_ALS_INT_100ms, cLTR239_ALS_RATE_500ms);
    Serial.print("[LTR329] MEAS REG: 0b");
    Serial.println(gLtr329.readMeasRate().raw, BIN);
    }

void loop()
    {
    float lux;
    lux = gLtr329.readLux();
    Serial.print("[LTR329] Lux: ");
    Serial.println(lux);
    }