/*

Module: MCCI_Catena_LTR329.h

Function:
        Definitions for the Catena library for the LITE ON LTR329 sensor.

Copyright and License:
        See accompanying LICENSE file.

Author:
        Pranau, MCCI Corporation   June 2022

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
    #define LTR329_I2C_ADDRESS 0x29 // 7-bit Address

    // Register Address
    #define LTR329_ADDR_ALS_CONTROL        0x80
    #define LTR329_ADDR_ALS_MEAS_RATE      0x85
    #define LTR329_ADDR_PART_ID            0x86
    #define LTR329_ADDR_MANUFAC_ID         0x87
    #define LTR329_ADDR_ALS_DATA_CH_1_0    0x88
    #define LTR329_ADDR_ALS_DATA_CH_1_1    0x89
    #define LTR329_ADDR_ALS_DATA_CH_0_0    0x8A
    #define LTR329_ADDR_ALS_DATA_CH_0_1    0x8B
    #define LTR329_ADDR_ALS_STATUS         0x8C

    // PFactoer for Calculating lux
    #define LTR329_PFACTOR                 1

    // ALS Gain Bit
    typedef enum
        {
        LTR329_ALS_GAIN_x1     = 0b000,
        LTR329_ALS_GAIN_x2     = 0b001,
        LTR329_ALS_GAIN_x4     = 0b010,
        LTR329_ALS_GAIN_x8     = 0b011,
        LTR329_ALS_GAIN_x48    = 0b110,
        LTR329_ALS_GAIN_x96    = 0b111,
        LTR329_ALS_GAIN_COUNT  = 8,
        } ALS_GAIN_Enum;

    // ALS Integration Time
    typedef enum
        {
        LTR329_ALS_INT_100ms   = 0b000,
        LTR329_ALS_INT_50ms    = 0b001,
        LTR329_ALS_INT_200ms   = 0b010,
        LTR329_ALS_INT_400ms   = 0b011,
        LTR329_ALS_INT_150ms   = 0b100,
        LTR329_ALS_INT_250ms   = 0b101,
        LTR329_ALS_INT_300ms   = 0b110,
        LTR329_ALS_INT_350ms   = 0b111,
        LTR329_ALS_INT_COUNT   = 8,
        } ALS_INT_Enum;

    // ALS Measuement Rate
    typedef enum
        {
        LTR329_ALS_RATE_50ms   = 0b000,
        LTR329_ALS_RATE_100ms  = 0b001,
        LTR329_ALS_RATE_200ms  = 0b010,
        LTR329_ALS_RATE_500ms  = 0b011,
        LTR329_ALS_RATE_1000ms = 0b100,
        LTR329_ALS_RATE_2000ms = 0b101,
        } ALS_MEAS_Enum;

    // Co-efficient for Calc lux
    const uint8_t ALS_GAIN[LTR329_ALS_GAIN_COUNT] =
        {
        1,      // LTR329_ALS_GAIN_x1
        2,      // LTR329_ALS_GAIN_x2
        4,      // LTR329_ALS_GAIN_x4
        8,      // LTR329_ALS_GAIN_x8
        0,      // Reserved
        0,      // Reserved
        48,     // LTR329_ALS_GAIN_x48
        96,     // LTR329_ALS_GAIN_x96
        };

    const float ALS_INT[LTR329_ALS_INT_COUNT] =
        {
        1,      // LTR329_ALS_INT_100ms
        0.5,    // LTR329_ALS_INT_50ms
        2,      // LTR329_ALS_INT_200ms
        4,      // LTR329_ALS_INT_400ms
        1.5,    // LTR329_ALS_INT_150ms
        2.5,    // LTR329_ALS_INT_250ms
        3,      // LTR329_ALS_INT_300ms
        3.5,    // LTR329_ALS_INT_350ms
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
                LTR329 = 0x29,
                };

            // the type for pin assignments, in case the ready pin is used
            using Pin_t = std::int8_t;

            // constructor
            cLTR329(TwoWire &wire, Address Address = Address::LTR329, Pin_t pinReady = -1)
                : m_wire(&wire)
                , m_address(Address)
                , m_pinReady(pinReady)
                {}

            // the errors
            enum class Error : std::uint8_t
                {
                Success = 0,
                NoWire,
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

            void writetControl(bool m_isActiveMode, ALS_GAIN_Enum m_gain);
            ALS_CONTR_REG readControl();

            void writeMeasRate(ALS_INT_Enum m_intTime, ALS_MEAS_Enum m_measRate);
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
            ALS_GAIN_Enum gain = LTR329_ALS_GAIN_x1;
            ALS_INT_Enum intTime = LTR329_ALS_INT_100ms;
            ALS_MEAS_Enum measRate = LTR329_ALS_RATE_500ms;
        };

    } // namespace McciCatenaLtr329

#endif // _MCCI_CATENA_LTR329_H_