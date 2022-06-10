/*

Module: ltr329_simple.ino

Function:
        Simple example for LTR329 Optical sensor.

Copyright and License:
        See accompanying LICENSE file.

Author:
        Pranau R, MCCI Corporation   June 2022

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

/*
Name: setup()

Function:
        Initializes LTR329 Light Sensor

Definition:
        void setup (void);

Returns:
        Functions returning type void: nothing.
*/

void setup()
    {
    Serial.begin(115200);

    while (!Serial);
    Serial.println("LTR329 Simple Test!");
    Serial.print("");

    // Start LTR329
    if (! gLtr329.begin())
        {
        Serial.println("gLtr329.begin() failed");
        }

    Serial.println("Found Sensor: LTR-329ALS-01");
    char tempString[64] = {};
    sprintf(tempString, "  Manufacture ID:         0x%02x", gLtr329.readManufacturerId());
    Serial.println(tempString);
    sprintf(tempString, "  Part Number:            0x%02x", gLtr329.readPartNumber());
    Serial.println(tempString);
    sprintf(tempString, "  Revision ID:            0x%02x", gLtr329.readRevisionId());
    Serial.println(tempString);
    Serial.print("  Status Register:        0b");
    Serial.println(gLtr329.readStatus().raw, BIN);

    gLtr329.writetControl(true, cLTR329_ALS_GAIN_x8);
    Serial.print("  Control Register:       0b");
    Serial.println(gLtr329.readControl().raw, BIN);

    gLtr329.writeMeasRate(cLTR329_ALS_INT_100ms, cLTR329_ALS_RATE_500ms);
    Serial.print("  Measure Rate Register:  0b");
    Serial.println(gLtr329.readMeasRate().raw, BIN);
    }

/*
Name:   loop()

Function:
        Performs Lux calculation and display the Lux.

Definition:
        void loop (void);

Returns:
        Functions returning type void: nothing.
*/

void loop()
    {
    auto lux = gLtr329.readLux();
    Serial.print("Lux = ");
    Serial.println(lux);
    delay (30000);
    }