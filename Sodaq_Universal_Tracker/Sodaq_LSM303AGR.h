/*
Copyright (c) 2018, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SODAQ_LSM303AGR_H_
#define SODAQ_LSM303AGR_H_

#include <stdint.h>
#include <Wire.h>

#define SODAQ_LSM303AGR_ACCEL_ADDRESS 0b0011001
#define SODAQ_LSM303AGR_MAG_ADDRESS 0b0011110

class Sodaq_LSM303AGR
{
public:

    enum Register {
        STATUS_REG_AUX = 0x07,
        OUT_TEMP_L_A = 0x0C,
        OUT_TEMP_H_A = 0x0D,
        INT_COUNTER_REG_A = 0x0E,
        WHO_AM_I_A = 0x0F,
        TEMP_CFG_REG_A = 0x1F,
        CTRL_REG1_A = 0x20,
        CTRL_REG2_A = 0x21,
        CTRL_REG3_A = 0x22,
        CTRL_REG4_A = 0x23,
        CTRL_REG5_A = 0x24,
        CTRL_REG6_A = 0x25,
        REFERENCE_DATACAPTURE_A = 0x26,
        STATUS_REG_A = 0x27,
        OUT_X_L_A = 0x28,
        OUT_X_H_A = 0x29,
        OUT_Y_L_A = 0x2A,
        OUT_Y_H_A = 0x2B,
        OUT_Z_L_A = 0x2C,
        OUT_Z_H_A = 0x2D,
        FIFO_CTRL_REG_A = 0x2E,
        FIFO_SRC_REG_A = 0x2F,
        INT1_CFG_A = 0x30,
        INT1_SRC_A = 0x31,
        INT1_THS_A = 0x32,
        INT1_DURATION_A = 0x33,
        INT2_CFG_A = 0x34,
        INT2_SRC_A = 0x35,
        INT2_THS_A = 0x36,
        INT2_DURATION_A = 0x37,
        CLICK_CFG_A = 0x38,
        CLICK_SRC_A = 0x39,
        CLICK_THS_A = 0x3A,
        TIME_LIMIT_A = 0x3B,
        TIME_LATENCY_A = 0x3C,
        TIME_WINDOW_A = 0x3D,
        Act_THS_A = 0x3E,
        Act_DUR_A = 0x3F,
        OFFSET_X_REG_L_M = 0x45,
        OFFSET_X_REG_H_M = 0x46,
        OFFSET_Y_REG_L_M = 0x47,
        OFFSET_Y_REG_H_M = 0x48,
        OFFSET_Z_REG_L_M = 0x49,
        OFFSET_Z_REG_H_M = 0x4A,
        WHO_AM_I_M = 0x4F,
        CFG_REG_A_M = 0x60,
        CFG_REG_B_M = 0x61,
        CFG_REG_C_M = 0x62,
        INT_CTRL_REG_M = 0x63,
        INT_SOURCE_REG_M = 0x64,
        INT_THS_L_REG_M = 0x65,
        INT_THS_H_REG_M = 0x66,
        STATUS_REG_M = 0x67,
        OUTX_L_REG_M = 0x68,
        OUTX_H_REG_M = 0x69,
        OUTY_L_REG_M = 0x6A,
        OUTY_H_REG_M = 0x6B,
        OUTZ_L_REG_M = 0x6C,
        OUTZ_H_REG_M = 0x6D,
    };

    enum RegisterBits {
        // CTRL_REG1_A
        ODR0 = 4,
        LPen = 3,
        Zen = 2,
        Yen = 1,
        Xen = 0,

        // CTRL_REG3_A
        INT1_CLICK = 7,
        INT1_AOI1 = 6,
        INT1_AOI2 = 5,
        INT1_DRDY1 = 4,
        INT1_DRDY2 = 3,
        INT1_WTM = 2,
        INT1_OVERRUN = 1,

        // CTRL_REG4_A
        BDU = 7,
        BLE = 6,
        FS0 = 4,
        HR = 3,
        ST0 = 1,
        SPI_ENABLE = 0,

        // CTRL_REG5_A
        BOOT = 7,
        FIFO_EN = 6,
        LIR_IG1 = 3,
        D4D_IG1 = 2,
        LIR_IG2 = 1,
        D4D_IG2 = 0,

        // CTRL_REG6_A
        INT2_CLICK_EN = 7,
        I2_INT1 = 6,
        I2_INT2 = 5,
        BOOT_I2 = 4,
        P2_ACT = 3,
        H_LACTIVE = 1,

        // TEMP_CFG_REG_A
        TEMP_EN1 = 7,
        TEMP_EN0 = 6,

    };

    enum MagRegisterBits {
        // CFG_REG_A_M
        MD0 = 0,
        MD1 = 1,
        MagODR0 = 2,
        LP = 4,
        SOFT_RST = 5,
        REBOOT = 6,
        COMP_TEMP_EN = 7,

        // CFG_REG_B_M
        LPF = 0,
        OFF_CANC = 1,
        Set_FREQ = 2,
        INT_on_DataOFF = 3,
        OFF_CANC_ONE_SHOT = 4,

        // CFG_REG_C_M
        INT_MAG = 0,
        Self_test = 1,
        MagBLE = 3,
        MagBDU = 4,
        I2C_DIS = 5,
        INT_MAG_PIN = 6,

        // INT_CTRL_REG_M
        IEN = 0,
        IEL = 1,
        IEA = 2,
        ZIEN = 5,
        YIEN = 6,
        XIEN = 7,

        // INT_SOURCE_REG_M
        INT = 0,
        MROI = 1,
        X_TH_S_Z = 2,
        N_TH_S_Y = 3,
        N_TH_S_X = 4,

        P_TH_S_Z = 5,
        P_TH_S_Y = 6,
        P_TH_S_X = 7,

        // STATUS_REG_M
        xda = 0,
        yda = 1,
        zda = 2,

        Zyxda = 3,

        Magxor = 4,
        Magyor = 5,
        Magzor = 5,

        Zyxor = 6,
    };

    // the values of the following enum are 0b(LPen,HR)
    enum AccelerometerMode {
        LowPowerMode = 0b10,
        NormalMode = 0b00,
        HighResMode = 0b01
    };

    enum MagnetometerMode {
        MagLowPowerMode = 0b0,
        MagHighResMode = 0b1
    };

    enum AccelerometerODR {
        PowerDown = 0b000,
        HrNormalLowPower1Hz = 0b0001,
        HrNormalLowPower10Hz = 0b0010,
        HrNormalLowPower25Hz = 0b0011,
        HrNormalLowPower50Hz = 0b0100,
        HrNormalLowPower100Hz = 0b0101,
        HrNormalLowPower200Hz = 0b0110,
        HrNormalLowPower400Hz = 0b0111,
        LowPower1k6Hz = 0b1000,
        HrNormal1k344LowPower5k376Hz = 0b1001
    };

    enum MagnetometerODR {
        Hz10 = 0b00,
        Hz20 = 0b01,
        Hz50 = 0b10,
        Hz100 = 0b11,
    };

    enum MagnetometerSystemMode {
        Continuous = 0b00,
        Single = 0b01,
        IdleMode = 0b10,
    };

    enum Axes {
        NoAxis = 0,
        X = 0b001,
        Y = 0b010,
        Z = 0b100,
        XY = X | Y,
        XZ = X | Z,
        YZ = Y | Z,
        XYZ = X | Y | Z
    };

    enum MagAxes {
        MagX = 0b100,
        MagY = 0b010,
        MagZ = 0b001
    };

    enum Scale {
        Scale2g = 0,
        Scale4g = 0b01,
        Scale8g = 0b10,
        Scale16g = 0b11
    };

    enum AxesEvents {
        ZHigh = 0b00100000,
        ZLow = 0b00010000,
        YHigh = 0b00001000,
        YLow = 0b00000100,
        XHigh = 0b00000010,
        XLow = 0b00000001
    };

    enum InterruptMode {
        OrCombination = 0b00000000,
        MovementRecognition = 0b01000000,
        AndCombination = 0b10000000,
        PositionRecognition = 0b11000000
    };

    Sodaq_LSM303AGR(TwoWire& wire = Wire, uint8_t accelAddress = SODAQ_LSM303AGR_ACCEL_ADDRESS, uint8_t magAddress = SODAQ_LSM303AGR_MAG_ADDRESS);

    bool checkWhoAmI();
    int8_t getTemperature();
    void enableAccelerometer(AccelerometerMode mode = NormalMode, AccelerometerODR odr = HrNormalLowPower25Hz, Axes axes = XYZ, Scale scale = Scale2g, bool isTemperatureOn = true);
    void disableAccelerometer();
    void rebootAccelerometer();

    void enableMagnetometer(MagnetometerMode mode = MagLowPowerMode, MagnetometerODR odr = Hz10, MagnetometerSystemMode systemMode = Single, bool compensateTemp = true, bool enableLPF = true);
    void disableMagnetometer();
    void rebootMagnetometer();

    void enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode = MovementRecognition);
    void disableInterrupt1();
    void enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode = MovementRecognition);
    void disableInterrupt2();
    void enableMagnetometerInterrupt(uint8_t axesEvents, double threshold, bool highOnInterrupt = true);
    void disableMagnetometerInterrupt();

    double getX() { return getGsFromScaledValue(readAccelRegister16Bits(Sodaq_LSM303AGR::OUT_X_L_A)); };
    double getY() { return getGsFromScaledValue(readAccelRegister16Bits(Sodaq_LSM303AGR::OUT_Y_L_A)); };
    double getZ() { return getGsFromScaledValue(readAccelRegister16Bits(Sodaq_LSM303AGR::OUT_Z_L_A)); };

    double getMagX() { return getMagFromScaledValue(readMagRegister16Bits(Sodaq_LSM303AGR::OUTX_L_REG_M)); };
    double getMagY() { return getMagFromScaledValue(readMagRegister16Bits(Sodaq_LSM303AGR::OUTY_L_REG_M)); };
    double getMagZ() { return getMagFromScaledValue(readMagRegister16Bits(Sodaq_LSM303AGR::OUTZ_L_REG_M)); };
protected:
    TwoWire& _wire;
    uint8_t _accelAddress;
    uint8_t _magAddress;
    Scale _accelScale = Scale::Scale2g;
    AccelerometerMode _accelMode = AccelerometerMode::LowPowerMode;

    void setLPF(bool enabled);

    double getGsFromScaledValue(int16_t value);
    uint8_t getScaledInterruptThreshold(double threshold);
    double getMagFromScaledValue(int16_t value);

    uint8_t readRegister(uint8_t deviceAddress, uint8_t reg);
    uint16_t readRegister16Bits(uint8_t deviceAddress, uint8_t reg);
    void writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value);
    void setRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue);
    void unsetRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue);

    uint8_t readAccelRegister(uint8_t reg) { return readRegister(_accelAddress, reg); };
    uint16_t readAccelRegister16Bits(uint8_t reg) { return readRegister16Bits(_accelAddress, reg); };
    void writeAccelRegister(uint8_t reg, uint8_t value) { writeRegister(_accelAddress, reg, value); };
    void setAccelRegisterBits(Register reg, uint8_t byteValue) { setRegisterBits(_accelAddress, reg, byteValue); };
    void unsetAccelRegisterBits(Register reg, uint8_t byteValue) { unsetRegisterBits(_accelAddress, reg, byteValue); };

    uint8_t readMagRegister(uint8_t reg) { return readRegister(_magAddress, reg); };
    uint16_t readMagRegister16Bits(uint8_t reg) { return readRegister16Bits(_magAddress, reg); };
    void writeMagRegister(uint8_t reg, uint8_t value) { writeRegister(_magAddress, reg, value); };
    void setMagRegisterBits(Register reg, uint8_t byteValue) { setRegisterBits(_magAddress, reg, byteValue); };
    void unsetMagRegisterBits(Register reg, uint8_t byteValue) { unsetRegisterBits(_magAddress, reg, byteValue); };
};

#endif
