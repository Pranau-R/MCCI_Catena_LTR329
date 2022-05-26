/*

Module: ltr329_simple.ino

Function:
        Simple example for LTR329 Optical sensor.

Copyright and License:
        See accompanying LICENSE file.
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

cLTR329 gLtr329 {Wire};

/****************************************************************************\
|
|   Code.
|
\****************************************************************************/

void setup()
    {
    Serial.begin(115200);

    while (!Serial);
    Serial.println("LTR329 Simple Test!");
    // Start LTR329

    if (! gLtr329.begin())
        {
        printFailure("gLtr329.begin() failed");
        }

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