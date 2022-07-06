 # MCCI Catena&reg; Arduino Library for LITE ON LTR-329ALS-01 Optical Sensor

 This library provides a simple interface to LITE ON LTR-329ALS-01 Optical Sensor. There are no dependencies on MCCI hardware; this should work equally well with Adafruit breakout boards, etc. 

See [Meta](#meta) for information on getting sensors from MCCI

## Contents
- [Library Contents](#library-Contents)
    - [Files](#files)
- [Installation](#installation)
	- [Installing Manually With Zip](#installing-manually-with-zip)
	- [Installing with the IDE](#installing-with-the-ide)
- [Using the Library](#using-the-library)
	- [Header file](#header-file)
	- [Namespaces](#namespaces)
	- [Declare Sensor Objects](#declare-sensor-objects)
	- [Preparing for use](#preparing-for-use)
- [Meta](#meta)
	- [License](#license)
	- [Support Open Source Hardware and Software](#support-open-source-hardware-and-software)
	- [Trademarks](#trademarks)

## Library Contents

### Files

This list is not exhaustive, but highlights some of the more important files.

File | Description
-----|------------
`LICENSE.md` | GNU License file
`examples/*` | Sample sketches to implement miscellaneous settings.
`src/MCCI_Catena_LTR329.h` | The library header file
`src/lib/MCCI_Catena_LTR329.cpp` | The main source file for the library

## Installation

### Installing Manually With Zip

- Download the latest ZIP version from the [release page]()
- Either unzip manually into your Arduino/Libraries directory, or use the IDE and select `Sketch > Include Library > Add .ZIP Library`.

### Installing with the IDE

- This library is published as an official Arduino library. So you can install from the IDE using `Sketch > Include Library > Manage Libraries...` to open the library manager, then search for `MCCI` and select this library from the list.

## Using the Library

### Header file

```c++
#include <MCCI_Catena_LTR329.h>
```

### Library Dependencies

None, beyond the normal Arduino library `<Wire.h>`.  It can be used with [Catena-Arduino-Platform](https://github.com/mcci-catena/Catena-Arduino-Platform), but it doesn't require it.

### Namespaces

```c++
using namespace McciCatenaLtr329;
```

### Declare Sensor Objects

```c++
// declare an instance of a sensor using TwoWire interface Wire,
// the specified address, and the specified digitial I/O for the RDY pin.
// The only defined address is:
//    cLTR329::Address::LTR329 (0x61)
// If RDY pin is not used, supply -1 as the default.
// The default for the interface is &Wire, and the default i2c_address is 0x29H.
cLTR329 myLtr(&Wire, cLTR329::Address::LTR329, -1);
```

You need to declare one `cLTR329` instance for each sensor. (Because all LTR329s have the same I2C address, you must either use multiple `TwoWire` busses or must implement some form of multiplexing.)

### Preparing for use

Use the `begin()` method to prepare for use.

```c++
bool cLTR329::begin();
```

It returns `true` for success, `false` for failure.

### Reading Lux

Use the `readLux()` method to read the Lux.

```c++
float cLTR329::readLux()
```

It returns a `floating` integer which is nothing but Lux.

### Reset Sensor

Use the `reset()` method to reset the sensor.
```c++
void cLTR329::reset()
```

### Write the control register

```c++
void cLTR329::writetControl(bool isActiveMode, ALS_GAIN_Enum gain)
```

It writes the control register of a sensor by passing a boolean control mode and a gain. 

### Read the control register

```c++
ALS_CONTR_REG cLTR329::readControl()
```

It is used to read the current control register of the sensor.

### Write the measure rate register of sensor

```c++
void cLTR329::writeMeasRate(ALS_INT_Enum intTime, ALS_MEAS_Enum measRate)
```

It writes the measure rate register of a sensor by passing a measure rate and a count.

### Read the measure rate register of sensor


```c++
ALS_MEAS_RATE_REG cLTR329::readMeasRate()
```

It is used to read the current measure rate register of the sensor.

### Read the status register

Use the `readStatus()` method to read the status registor of the sensor.

```c++
ALS_PS_STATUS_REG cLTR329::readStatus()
```

## Meta

### License

This repository is released under the [MIT](./LICENSE) license. Commercial licenses are also available from MCCI Corporation.

### Support Open Source Hardware and Software

MCCI invests time and resources providing this open source code, please support MCCI and open-source hardware by purchasing products from MCCI, Adafruit and other open-source hardware/software vendors!

For information about MCCI's products, please visit [store.mcci.com](https://store.mcci.com/).

### Trademarks

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. All other marks are the property of their respective owners.