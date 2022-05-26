/*

Module: MCCI_Catena_LTR329.h

Function:
    Definitions for the Catena library for the LITE ON LTR329 sensor.

Copyright and License:
    This file copyright (C) 2020 by

        MCCI Corporation
        3520 Krums Corners Road
        Ithaca, NY  14850

    See accompanying LICENSE file for copyright and license information.

Author:
    Pranau, MCCI Corporation   May 2022

*/

# ifndef _MCCI_CATENA_LTR329_H_
# define _MCCI_CATENA_LTR329_H_
# pragma once

#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>

namespace McciCatenaLtr329
    {
    /// create a version number for comparison
    static constexpr std::uint32_t
    makeVersion(
        std::uint8_t major, std::uint8_t minor, std::uint8_t patch, std::uint8_t local = 0
        )
        {
        return ((std::uint32_t)major << 24) | ((std::uint32_t)minor << 16) | ((std::uint32_t)patch << 8) | (std::uint32_t)local;
        }

    /// extract major number from version
    static constexpr std::uint8_t
    getMajor(std::uint32_t v)
        {
        return std::uint8_t(v >> 24u);
        }

    /// extract minor number from version
    static constexpr std::uint8_t
    getMinor(std::uint32_t v)
        {
        return std::uint8_t(v >> 16u);
        }

    /// extract patch number from version
    static constexpr std::uint8_t
    getPatch(std::uint32_t v)
        {
        return std::uint8_t(v >> 8u);
        }

    /// extract local number from version
    static constexpr std::uint8_t
    getLocal(std::uint32_t v)
        {
        return std::uint8_t(v);
        }

    /// version of library, for use by clients in static_asserts
    static constexpr std::uint32_t kVersion = makeVersion(0,1,0,0);

    // Device Address
    #define cLTR239_I2C_ADDRESS 0x29 // 7-bit Address

    // Register Address
    #define cLTR239_ADDR_ALS_CONTROL        0x80
    #define cLTR239_ADDR_ALS_MEAS_RATE      0x85
    #define cLTR239_ADDR_PART_ID            0x86
    #define cLTR239_ADDR_MANUFAC_ID         0x87
    #define cLTR239_ADDR_ALS_DATA_CH_1_0    0x88
    #define cLTR239_ADDR_ALS_DATA_CH_1_1    0x89
    #define cLTR239_ADDR_ALS_DATA_CH_0_0    0x8A
    #define cLTR239_ADDR_ALS_DATA_CH_0_1    0x8B
    #define cLTR239_ADDR_ALS_STATUS         0x8C
    // PFactoer for Calculating lux
    #define cLTR239_PFACTOR                 1

    // ALS Gain Bit
    typedef enum
        {
        cLTR239_ALS_GAIN_x1     = 0b000,
        cLTR239_ALS_GAIN_x2     = 0b001,
        cLTR239_ALS_GAIN_x4     = 0b010,
        cLTR239_ALS_GAIN_x8     = 0b011,
        cLTR239_ALS_GAIN_x48    = 0b110,
        cLTR239_ALS_GAIN_x96    = 0b111,
        cLTR239_ALS_GAIN_COUNT  = 8,
        } ALS_GAIN_Enum;

    // ALS Integration Time
    typedef enum
        {
        cLTR239_ALS_INT_100ms   = 0b000,
        cLTR239_ALS_INT_50ms    = 0b001,
        cLTR239_ALS_INT_200ms   = 0b010,
        cLTR239_ALS_INT_400ms   = 0b011,
        cLTR239_ALS_INT_150ms   = 0b100,
        cLTR239_ALS_INT_250ms   = 0b101,
        cLTR239_ALS_INT_300ms   = 0b110,
        cLTR239_ALS_INT_350ms   = 0b111,
        cLTR239_ALS_INT_COUNT   = 8,
        } ALS_INT_Enum;

    // ALS Measuement Rate
    typedef enum
        {
        cLTR239_ALS_RATE_50ms   = 0b000,
        cLTR239_ALS_RATE_100ms  = 0b001,
        cLTR239_ALS_RATE_200ms  = 0b010,
        cLTR239_ALS_RATE_500ms  = 0b011,
        cLTR239_ALS_RATE_1000ms = 0b100,
        cLTR239_ALS_RATE_2000ms = 0b101,
        } ALS_MEAS_Enum;

    // Co-efficient for Calc lux
    const uint8_t ALS_GAIN[cLTR239_ALS_GAIN_COUNT] =
        {
        1,      // cLTR239_ALS_GAIN_x1
        2,      // cLTR239_ALS_GAIN_x2
        4,      // cLTR239_ALS_GAIN_x4
        8,      // cLTR239_ALS_GAIN_x8
        0,      // Reserved
        0,      // Reserved
        48,     // cLTR239_ALS_GAIN_x48
        96,     // cLTR239_ALS_GAIN_x96
        };

    const float ALS_INT[cLTR239_ALS_INT_COUNT] =
        {
        1,      // cLTR239_ALS_INT_100ms
        0.5,    // cLTR239_ALS_INT_50ms
        2,      // cLTR239_ALS_INT_200ms
        4,      // cLTR239_ALS_INT_400ms
        1.5,    // cLTR239_ALS_INT_150ms
        2.5,    // cLTR239_ALS_INT_250ms
        3,      // cLTR239_ALS_INT_300ms
        3.5,    // cLTR239_ALS_INT_350ms
        };

    typedef union
        {
        uint8_t raw; 
        struct
            {
            bool activeMode: 1;
            bool resetState: 1;
            uint8_t gain: 3;
            uint8_t reserved: 3;
            };
        } ALS_CONTR_REG;

    typedef union
        {
        uint8_t raw;
        struct
            {
            uint8_t measRate: 3;
            uint8_t intTime: 3;
            uint8_t reserved: 2;
            };
        } ALS_MEAS_RATE_REG;

    typedef union
        {
        uint8_t raw;
        struct
            {
            uint8_t revId: 4;
            uint8_t partNum: 4;
            };
        } PART_ID_REG;

    typedef union
        {
        uint8_t raw;
        struct
            {
            uint8_t manId: 8;
            };
        } MANUFAC_ID_REG;

    typedef union
        {
        uint8_t raw;
        struct
            {
            uint8_t reseved0: 2;
            bool isNewData: 1;
            uint8_t reseved1: 1;
            uint8_t gainRange: 3;
            bool isInValid: 1;
            };
        } ALS_PS_STATUS_REG;

    class cLTR329
        {
        public:
            // the address type:
            enum class Address : std::int8_t
                {
                Error = -1,
                LTR329 = 0x61,
                };

            // the type for pin assignments, in case the ready pin is used
            using Pin_t = std::int8_t;

            // constructor
            cLTR329(TwoWire &wire, Address Address = Address::LTR329, Pin_t pinReady = -1)
                : m_wire(&wire)
                , m_address(Address)
                , m_pinReady(pinReady)
                {}

            enum class Command : std::uint16_t
                {
                // sorted in ascending numerical order.
                StartContinuousMeasurement              =   0x0036,     // 1.4.1; takes 16-bit arg (pressure)
                StopContinuosMeasurement                =   0x0104,     // 1.4.2; no argument
                GetDataReady                            =   0x0202,     // 1.4.4; no argument
                ReadMeasurement                         =   0x0300,     // 1.4.5; no argument (returns 6*3 bytes: CO2, T, RH)
                //SetMeasurementInterval                  =   0x4600,     // 1.4.3; set/get measurement interval; takes 16 bit arg (seconds)
                //AltitudeCompensation                    =   0x5102,     // 1.4.8; 16-bit argument (meters above sea level)
                //SetForcedRecalibration                  =   0x5204,     // 1.4.6; 16-bit CO2 concentration (ppm)
                //EnableAutoSelfCal                       =   0x5306,     // 1.4.6; 16-bit bool (enable/disable)
                //SetTemperatureOffset                    =   0x5403,     // 1.4.7; 16-bit temperature in 0.01 deg C
                //ReadFirmwareVersion                     =   0xD100,     // 1.4.9; no argument (returns 3 bytes)
                SoftReset                               =   0xD304,     // 1.4.10; no argument
                };

            // the errors
            enum class Error : std::uint8_t
                {
                Success = 0,
                NoWire,
                CommandWriteFailed,
                CommandWriteBufferFailed,
                InternalInvalidParameter,
                I2cReadShort,
                I2cReadRequest,
                I2cReadLong,
                Busy,
                NotMeasuring,
                Crc,
                Uninitialized,
                InvalidParameter,
                InternalInvalidState,
                SensorUpdateFailed,
                };

            // state of the measurement enging
            enum class State : std::uint8_t
                {
                Uninitialized,      /// this->begin() has never succeeded.
                End,                /// this->begin() succeeded, followed by this->end()
                Initial,            /// initial after begin [indeterminate]
                Idle,               /// idle (not measuring)
                Triggered,          /// continuous measurement running, no data available.
                Ready,              /// continuous measurement running, data availble.
                };

        private: 

            // this is internal -- centralize it but require that clients call the
            // public method (which centralizes the strings and the search)
            static constexpr const char * const m_szErrorMessages =
                "Success\0"
                "NoWire\0"
                "CommandWriteFailed\0"
                "CommandWriteBufferFailed\0"
                "InternalInvalidParameter\0"
                "I2cReadShort\0"
                "I2cReadRequest\0"
                "I2cReadLong\0"
                "Busy\0"
                "NotMeasuring\0"
                "Crc\0"
                "Uninitialized\0"
                "InvalidParmaeter\0"
                "InternalInvalidState\0"
                ;

            // this is internal -- centralize it but require that clients call the
            // public method (which centralizes the strings and the search)
            static constexpr const char * const m_szStateNames =
                "Uninitialized" "\0"
                "End"           "\0"
                "Initial"       "\0"
                "Idle"          "\0"
                "Triggered"     "\0"
                "Ready"         "\0"
                ;

        public:
            cLTR329(void);

            virtual ~cLTR329();

            bool begin();

            void reset();

            float readLux();

            void writetControl(bool _isActiveMode, ALS_GAIN_Enum _gain);
            ALS_CONTR_REG readControl();

            void writeMeasRate(ALS_INT_Enum _intTime, ALS_MEAS_Enum _measRate);
            ALS_MEAS_RATE_REG readMeasRate();

            uint8_t readPartNumber();
            uint8_t readRevisionId();
            uint8_t readManufacturerId();
                
            ALS_PS_STATUS_REG readStatus();

            Error getLastError() const
                {
            return this->m_lastError;
                }
            bool setLastError(Error e)
                {
                this->m_lastError = e;
                return e == Error::Success;
                }
            static const char *getErrorName(Error e);
            const char *getLastErrorName() const
                {
                return getErrorName(this->m_lastError);
                }
            static const char *getStateName(State s);
            const char *getCurrentStateName() const
                {
                return getStateName(this->getState());
                }
            bool isRunning() const
                {
                return this->m_state > State::End;
                }
            State getState() const
                {
                return this->m_state;
                }

        private:
            Error m_lastError;
            TwoWire *m_wire;
            Pin_t m_pinReady;
            Address m_address;
            State m_state;
            uint8_t readByte(uint8_t addr);
            uint16_t readAlsData(uint8_t ch);

            void writeByte(uint8_t addr, uint8_t data);

            bool isActiveMode = false;
            ALS_GAIN_Enum gain = cLTR239_ALS_GAIN_x1;
            ALS_INT_Enum intTime = cLTR239_ALS_INT_100ms;
            ALS_MEAS_Enum measRate = cLTR239_ALS_RATE_500ms;
        };

    } // namespace McciCatenaLtr329

#endif // _MCCI_CATENA_LTR329_H_