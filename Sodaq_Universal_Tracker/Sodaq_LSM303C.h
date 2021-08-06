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

#ifndef SODAQ_LSM303C_H_
#define SODAQ_LSM303C_H_

#include <Wire.h>
#include <stdint.h>


#define SODAQ_LSM303C_ACCEL_ADDRESS 0b0011101
#define SODAQ_LSM303C_MAG_ADDRESS   0b0011110

#define WHOAMI_ID_A_LSM303C 0b01000001
#define WHOAMI_ID_M_LSM303C 0b00111101

class Sodaq_LSM303C {
public:
    enum Register {
        WHO_AM_I_A = 0x0F,
        ACT_THS_A = 0x1E,
        ACT_DUR_A = 0x1F,
        CTRL_REG1_A = 0x20,
        CTRL_REG2_A = 0x21,
        CTRL_REG3_A = 0x22,
        CTRL_REG4_A = 0x23,
        CTRL_REG5_A = 0x24,
        CTRL_REG6_A = 0x25,
        CTRL_REG7_A = 0x26,
        STATUS_REG_A = 0x27,
        OUT_X_L_A = 0x28,
        OUT_X_H_A = 0x29,
        OUT_Y_L_A = 0x2A,
        OUT_Y_H_A = 0x2B,
        OUT_Z_L_A = 0x2C,
        OUT_Z_H_A = 0x2D,
        FIFO_CTRL = 0x2E,
        FIFO_SRC = 0x2F,
        IG_CFG1_A = 0x30,
        IG_SRC1_A = 0x31,
        IG_THS_X1_A = 0x32,
        IG_THS_Y1_A = 0x33,
        IG_THS_Z1_A = 0x34,
        IG_DUR1_A = 0x35,
        IG_CFG2_A = 0x36,
        IG_SRC2_A = 0x37,
        IG_THS2_A = 0x38,
        IG_DUR2_A = 0x39,
        XL_REFERENCE = 0x3A,
        XH_REFERENCE = 0x3B,
        YL_REFERENCE = 0x3C,
        YH_REFERENCE = 0x3D,
        ZL_REFERENCE = 0x3E,
        ZH_REFERENCE = 0x3F,
        WHO_AM_I_M = 0x0F,
        CTRL_REG1_M = 0x20,
        CTRL_REG2_M = 0x21,
        CTRL_REG3_M = 0x22,
        CTRL_REG4_M = 0x23,
        CTRL_REG5_M = 0x24,
        STATUS_REG_M = 0x27,
        OUT_X_L_M = 0x28,
        OUT_X_H_M = 0x29,
        OUT_Y_L_M = 0x2A,
        OUT_Y_H_M = 0x2B,
        OUT_Z_L_M = 0x2C,
        OUT_Z_H_M = 0x2D,
        TEMP_L_M = 0x2E,
        TEMP_H_M = 0x2F,
        INT_CFG_M = 0x30,
        INT_SRC_M = 0x31,
        INT_THS_L_M = 0x32,
        INT_THS_H_M = 0x33,
    };

    enum RegisterBits {
        // CTRL_REG1_A
        Xen = 0,
        Yen = 1,
        Zen = 2,
        BDU_A = 3,
        ODR0 = 4,
        ODR1 = 5,
        ODR2 = 6,
        HR = 7,

        // CTRL_REG2_A
        HPIS1 = 0,
        HPIS2 = 1,
        FDS = 2,
        HPM0 = 3,
        HPM1 = 4,
        DFC0 = 5,
        DFC1 = 6,

        // CTRL_REG3_A
        FIFO_EN = 7,
        STOP_FTH = 6,
        INT_XL_INACT = 5,
        INT_XL_IG2 = 4,
        INT_XL_IG1 = 3,
        INT_XL_OVR = 2,
        NT_XL_FTH = 1,
        NT_XL_DRDY = 0,

        // CTRL_REG4_A
        BW2 = 7,
        BW1 = 6,
        FS1A = 5,
        FS0A = 4,
        BW_SCALE_ODR = 3,
        IF_ADD_INC = 2,
        I2C_DISABLE_A = 1,
        ASIM = 0,

        // CTRL_REG5_A
        DEBUG = 7,
        SOFT_RESET = 6,
        DEC1 = 5,
        DEC0 = 4,
        ST2 = 3,
        ST1 = 2,
        H_LACTIVE = 1,
        PP_OD = 0,

        // CTRL_REG6_A
        BOOT = 7,

        // CTRL_REG7_A
        DCRM2 = 5,
        DCRM1 = 4,
        LIR2 = 3,
        LIR1 = 2,
        I4D_IG2 = 1,
        I4D_IG1 = 0,

        // STATUS_REG_A
        ZYXOR = 7,
        ZOR = 6,
        YOR = 5,
        XOR = 4,
        ZYXDA = 3,
        ZDA = 2,
        YDA = 1,
        XDA = 0,

        // IG_CFG1_A
        AOI = 7,
        A6D = 6,
        ZHIE = 5,
        ZLIE = 4,
        YHIE = 3,
        YLIE = 2,
        XHIE = 1,
        XLIE = 0,

        // IG_SRC1_A
        IA = 6,
        ZH = 5,
        ZL = 4,
        YH = 3,
        YL = 2,
        XH = 1,
        XL = 0,

        // IG_DUR1_A
        WAIT1 = 7,
        DUR1_6 = 6,
        DUR1_5 = 5,
        DUR1_4 = 4,
        DUR1_3 = 3,
        DUR1_2 = 2,
        DUR1_1 = 1,
        DUR1_0 = 0,
    };

    enum MagRegisterBits {
        // CTRL_REG1_M
        ST = 0,
        DO0 = 2,
        DO1 = 3,
        DO2 = 4,
        OM0 = 5,
        OM1 = 6,
        TEMP_EN = 7,

        // CTRL_REG2_M
        SOFT_RST = 2,
        REBOOT = 3,
        FS0M = 5,
        FS1M = 6,

        // CTRL_REG3_M
        MD0 = 0,
        MD1 = 1,
        MSIM = 2,
        LP = 5,
        I2C_DISABLE_M = 7,

        // CTRL_REG4_M
        BLE = 1,
        OMZ0 = 2,
        OMZ1 = 3,

        // CTRL_REG5_M
        BDU_M = 6,

        // STATUS_REG_M
        xda = 0,
        yda = 1,
        zda = 2,

        Zyxda = 3,

        Magxor = 4,
        Magyor = 5,
        Magzor = 6,

        Zyxor = 7,

        // INT_CFG_M
        IEN = 0,
        IEL = 1,
        IEA = 2,
        ZIEN = 5,
        YIEN = 6,
        XIEN = 7,

        // INT_SRC_M
        INT = 0,
        MROI = 1,
        NTH_Z = 2,
        NTH_Y = 3,
        NTH_X = 4,
        PTH_Z = 5,
        PTH_Y = 6,
        PTH_X = 7,

        // INT_THS_H_M
        THS14 = 6,
        THS13 = 5,
        THS12 = 4,
        THS11 = 3,
        THS10 = 2,
        THS9 = 1,
        THS8 = 0,

        // INT_THS_L_M 7
        THS7 = 7,
        THS6 = 6,
        THS5 = 5,
        THS4 = 4,
        THS3 = 3,
        THS2 = 2,
        THS1 = 1,
        THS0 = 0,
    };

    // the values of the following enum are 0b(LPen,HR)
    enum AccelerometerMode { LowPowerMode = 0b10, NormalMode = 0b00, HighResMode = 0b01 };

    enum MagnetometerMode { MagLowPowerMode = 0b0, MagHighResMode = 0b1 };

    enum AccelerometerODR {
        PowerDown_A = 0b000,
        HrNormalLowPower10Hz = 0b0001,
        HrNormalLowPower50Hz = 0b0010,
        HrNormalLowPower100Hz = 0b0011,
        HrNormalLowPower200Hz = 0b0100,
        HrNormalLowPower400Hz = 0b0101,
        HrNormalLowPower800Hz = 0b0110,
    };

    enum MagnetometerODR {
        Hz0_625 = 0b000,
        Hz1_25 = 0b001,
        Hz2_5 = 0b010,
        Hz5 = 0b011,
        Hz10 = 0b100,
        Hz20 = 0b101,
        Hz40 = 0b110,
        Hz80 = 0b111,
    };

    enum MagnetometerSystemMode { MagSysLowPowerMode = 0b00, MagSysMedPerfMode = 0b01, MagSysHighPerfMode = 0b10, MagSysUltraHighPerfMode = 0b11 };

    // enums hieronder chacken

    enum Axes { NoAxis = 0, X = 0b001, Y = 0b010, Z = 0b100, XY = X | Y, XZ = X | Z, YZ = Y | Z, XYZ = X | Y | Z };

    enum MagAxes { MagX = 0b100, MagY = 0b010, MagZ = 0b001 };

    enum Scale {
        Scale2g = 0,
        Scale4g = 0b10,
        Scale8g = 0b11,
    };

    enum AxesEvents { ZHigh = 0b00100000, ZLow = 0b00010000, YHigh = 0b00001000, YLow = 0b00000100, XHigh = 0b00000010, XLow = 0b00000001 };

    enum InterruptMode { OrCombination = 0b00000000, MovementRecognition = 0b01000000, AndCombination = 0b10000000, PositionRecognition = 0b11000000 };

    Sodaq_LSM303C(TwoWire& wire = Wire, uint8_t accelAddress = SODAQ_LSM303C_ACCEL_ADDRESS, uint8_t magAddress = SODAQ_LSM303C_MAG_ADDRESS);

    bool checkWhoAmI();
    int8_t getTemperature();
    void enableAccelerometer(
        AccelerometerMode mode = LowPowerMode,
        AccelerometerODR odr = HrNormalLowPower10Hz,
        Axes axes = XYZ,
        Scale scale = Scale2g,
        bool isTemperatureOn = true);
    void disableAccelerometer();
    void rebootAccelerometer();

    void enableMagnetometer(
        MagnetometerMode mode = MagLowPowerMode,
        MagnetometerODR odr = Hz10,
        MagnetometerSystemMode systemMode = MagSysLowPowerMode,
        bool enableLPF = true,
        bool isTemperatureOn = true);
    void disableMagnetometer();
    void rebootMagnetometer();

    void enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode = MovementRecognition);
    void disableInterrupt1();
    void enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode = MovementRecognition);
    void disableInterrupt2();
    void enableMagnetometerInterrupt(uint8_t axesEvents, double threshold, bool highOnInterrupt = true);
    void disableMagnetometerInterrupt();

    double getX() { return getGsFromScaledValue(readAccelRegister16Bits(Sodaq_LSM303C::OUT_X_L_A)); };
    double getY() { return getGsFromScaledValue(readAccelRegister16Bits(Sodaq_LSM303C::OUT_Y_L_A)); };
    double getZ() { return getGsFromScaledValue(readAccelRegister16Bits(Sodaq_LSM303C::OUT_Z_L_A)); };

    double getMagX() { return getMagFromScaledValue(readMagRegister16Bits(Sodaq_LSM303C::OUT_X_L_M)); };
    double getMagY() { return getMagFromScaledValue(readMagRegister16Bits(Sodaq_LSM303C::OUT_Y_L_M)); };
    double getMagZ() { return getMagFromScaledValue(readMagRegister16Bits(Sodaq_LSM303C::OUT_Z_L_M)); };

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
