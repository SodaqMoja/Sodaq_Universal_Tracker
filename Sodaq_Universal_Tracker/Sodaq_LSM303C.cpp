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

#include "Sodaq_LSM303C.h"
#include <Arduino.h>
#include <math.h>

Sodaq_LSM303C::Sodaq_LSM303C(TwoWire& wire, uint8_t accelAddress, uint8_t magAddress)
    : _wire(wire), _accelAddress(accelAddress), _magAddress(magAddress), _accelScale(Scale2g)
{
}

int8_t Sodaq_LSM303C::getTemperature()
{
    int16_t value = readMagRegister16Bits(TEMP_L_M) / 8 + 25.0f;
    return value;
}

double Sodaq_LSM303C::getGsFromScaledValue(int16_t value)
{
    switch (_accelScale) {
        case Scale::Scale2g:
            return value * 0.061f / 1000.0f;
        case Scale::Scale4g:
            return value * 0.122f / 1000.0f;
        case Scale::Scale8g:
            return value * 0.244f / 1000.0f;
        default:
            break;
    }

    return 0.0f;
}

double Sodaq_LSM303C::getMagFromScaledValue(int16_t value)
{
    return value * 0.58;
}

bool Sodaq_LSM303C::checkWhoAmI()
{
    return (readAccelRegister(WHO_AM_I_A) == WHOAMI_ID_A_LSM303C) && (readMagRegister(WHO_AM_I_M) == WHOAMI_ID_M_LSM303C);
}

void Sodaq_LSM303C::enableAccelerometer(AccelerometerMode mode, AccelerometerODR odr, Axes axes, Scale scale, bool isTemperatureOn)
{
    // set odr, mode, enabled axes
    // Note: the values of AccelerometerMode are 0b(LPen,HR)
    uint8_t ctrlReg1A = (odr << ODR0) | axes | bit(BDU_A);
    writeAccelRegister(CTRL_REG1_A, ctrlReg1A);

    uint8_t ctrlReg4A = readAccelRegister(CTRL_REG4_A);
    if ((mode & 0b01) == 0b01) // Highres
    {
        ctrlReg4A |= bit(HR);
    }
    else {
        ctrlReg4A &= ~bit(HR);
    }

    // set scale
    ctrlReg4A &= ~(0b11 << FS0A); // first unset all FS bits
    ctrlReg4A |= (scale << FS0A);

    // write the value to CTRL_REG4_A
    writeAccelRegister(CTRL_REG4_A, ctrlReg4A);

    _accelScale = scale;
    _accelMode = mode;
}

void Sodaq_LSM303C::enableMagnetometer(MagnetometerMode mode, MagnetometerODR odr, MagnetometerSystemMode systemMode, bool enableLPF, bool isTemperatureOn)
{
    // set odr, mode, systemMode
    writeMagRegister(CTRL_REG1_M, odr << DO0);
    setMagRegisterBits(CTRL_REG1_M, (systemMode << OM0));
    writeMagRegister(CTRL_REG2_M, 0);
    writeMagRegister(CTRL_REG3_M, 0);

    if (mode == MagLowPowerMode) {
        setMagRegisterBits(CTRL_REG3_M, bit(LP));
    }
    else {
        unsetMagRegisterBits(CTRL_REG3_M, bit(LP));
    }

    setLPF(enableLPF);

    if (isTemperatureOn) {
        // enable aux ADC and temperature sensor
        setMagRegisterBits(CTRL_REG1_M, bit(TEMP_EN));
    }
    else {
        // disable aux ADC and temperature sensor
        unsetMagRegisterBits(CTRL_REG1_M, bit(TEMP_EN));
    }
}

void Sodaq_LSM303C::setLPF(bool enabled)
{
    if (enabled) {
        setMagRegisterBits(CTRL_REG3_M, bit(LP));
    }
    else {
        unsetMagRegisterBits(CTRL_REG3_M, bit(LP));
    }
}

void Sodaq_LSM303C::disableAccelerometer()
{
    enableAccelerometer(LowPowerMode, PowerDown_A, NoAxis, _accelScale);
}

void Sodaq_LSM303C::disableMagnetometer()
{
    enableMagnetometer(MagHighResMode, Hz10, MagSysLowPowerMode, false, true);
}

void Sodaq_LSM303C::rebootAccelerometer()
{
    writeAccelRegister(CTRL_REG6_A, bit(BOOT));
}

void Sodaq_LSM303C::rebootMagnetometer()
{
    writeMagRegister(CTRL_REG2_M, bit(REBOOT));
}

void Sodaq_LSM303C::setRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value |= byteValue;
    writeRegister(deviceAddress, reg, value);
}

void Sodaq_LSM303C::unsetRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value &= ~byteValue;
    writeRegister(deviceAddress, reg, value);
}

uint8_t Sodaq_LSM303C::getScaledInterruptThreshold(double threshold)
{
    uint8_t divider = 0; // divider in mg

    switch (_accelScale) {
        case Scale::Scale2g:
            divider = 16;
            break;
        case Scale::Scale4g:
            divider = 32;
            break;
        case Scale::Scale8g:
            divider = 62;
            break;
        default:
            break;
    }

    return trunc(threshold * 8000.0f / divider);
}

void Sodaq_LSM303C::enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(IG_CFG1_A, interruptMode | (axesEvents & 0b00111111));
    writeAccelRegister(IG_DUR1_A, (duration & 0b01111111)); // time duration is INT1_DURATION_A/ODR

    writeAccelRegister(IG_THS_X1_A, getScaledInterruptThreshold(threshold)); // time duration is INT1_DURATION_A/ODR
    writeAccelRegister(IG_THS_Y1_A, getScaledInterruptThreshold(threshold)); // time duration is INT1_DURATION_A/ODR
    writeAccelRegister(IG_THS_Z1_A, getScaledInterruptThreshold(threshold)); // time duration is INT1_DURATION_A/ODR

    // disable latching
    unsetAccelRegisterBits(CTRL_REG7_A, bit(LIR1));
    unsetAccelRegisterBits(CTRL_REG7_A, bit(I4D_IG1));

    // enable interrupt generator 1 on INT1
    setAccelRegisterBits(CTRL_REG3_A, bit(INT_XL_IG1));
}

void Sodaq_LSM303C::disableInterrupt1()
{
    // disable interrupt generator 1
    unsetAccelRegisterBits(CTRL_REG3_A, bit(INT_XL_IG1));
}

void Sodaq_LSM303C::enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(IG_CFG2_A, interruptMode | (axesEvents & 0b00111111));
    writeAccelRegister(IG_DUR2_A, duration); // time duration is INT2_DURATION_A/ODR

    // enable interrupt generator 2 on INT2
    setAccelRegisterBits(CTRL_REG3_A, bit(INT_XL_IG2));
}

void Sodaq_LSM303C::disableInterrupt2()
{
    // disable interrupt generator 2 on INT2
    unsetAccelRegisterBits(CTRL_REG3_A, bit(INT_XL_IG2));
}

void Sodaq_LSM303C::enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt)
{
    // threshold needs to be positive, because mag checks interrupts for
    // -threshold and +threshold always
    if (threshold < 0) {
        threshold = -threshold;
    }

    // set axes
    writeMagRegister(INT_CFG_M, (magAxesEvents << ZIEN));

    // interrupt mode
    if (highOnInterrupt) {
        setMagRegisterBits(INT_CFG_M, bit(IEA));
    }
    else {
        unsetMagRegisterBits(INT_CFG_M, bit(IEA));
    }

    // set threshold registers
    int16_t ths = trunc(threshold / 1.5);
    writeMagRegister(INT_THS_L_M, ths & 0x00FF);
    writeMagRegister(INT_THS_H_M, (ths & 0xFF00) >> 8);

    // disable latching
    unsetMagRegisterBits(INT_CFG_M, bit(IEL));

    // enable mag interrupt
    setMagRegisterBits(INT_CFG_M, bit(IEN));
}

void Sodaq_LSM303C::disableMagnetometerInterrupt()
{
    // disable mag interrupt
    unsetMagRegisterBits(INT_CFG_M, bit(IEN));
}

uint8_t Sodaq_LSM303C::readRegister(uint8_t deviceAddress, uint8_t reg)
{
    _wire.beginTransmission(deviceAddress);
    _wire.write((uint8_t)reg);
    _wire.endTransmission();

    _wire.requestFrom(deviceAddress, 1);

    return _wire.read();
}

uint16_t Sodaq_LSM303C::readRegister16Bits(uint8_t deviceAddress, uint8_t reg)
{
    // TODO replace with request of 2 bytes?
    // TODO: don't we need BDU Here?
    uint16_t result = readRegister(deviceAddress, reg);
    result |= readRegister(deviceAddress, reg + 1) << 8;

    return result;
}

void Sodaq_LSM303C::writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(deviceAddress);

    _wire.write((uint8_t)reg);
    _wire.write((uint8_t)value);

    _wire.endTransmission();
}
