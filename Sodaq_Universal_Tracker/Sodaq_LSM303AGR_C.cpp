#include <Arduino.h>
#include <math.h>
#include "Sodaq_LSM303AGR_C.h"
#include <typeinfo>

#define ERRORMESSAGE    SerialUSB.println("Unsupported accelerometer");

Sodaq_LSM303AGR_C::Sodaq_LSM303AGR_C(TwoWire &wire) : _wire(wire)
{
    
}

void Sodaq_LSM303AGR_C::Init()
{
    Wire.beginTransmission(Sodaq_LSM303C_ACCEL_ADDRESS);
    if(Wire.endTransmission() == 0)
    {
        _lsm303Variant = LSM303C;
    }
    else
    {
        Wire.beginTransmission(Sodaq_LSM303AGR_ACCEL_ADDRESS);
        if(Wire.endTransmission() == 0)
        {
            _lsm303Variant = LSM303AGR;
        }
        else
        {
            _lsm303Variant = LSM303_Undefined;
        }
    }
}

int8_t Sodaq_LSM303AGR_C::getTemperature()
{
    int8_t temp;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            temp = lsm303agr.getTemperature();
        break;
        case LSM303C:
            temp = lsm303c.getTemperature();
        break;
        default:
            ERRORMESSAGE;
        break;
    }

    return temp;
}

void Sodaq_LSM303AGR_C::enableAccelerometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.enableAccelerometer();
        break;
        case LSM303C:
            lsm303c.enableAccelerometer();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::enableMagnetometer()
{
   switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.enableMagnetometer();
        break;
        case LSM303C:
            lsm303c.enableMagnetometer();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::disableAccelerometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.disableAccelerometer();
        break;
        case LSM303C:
            lsm303c.disableAccelerometer();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::disableMagnetometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.disableMagnetometer();
        break;
        case LSM303C:
            lsm303c.disableMagnetometer();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::rebootAccelerometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.rebootAccelerometer();
        break;
        case LSM303C:
            lsm303c.rebootAccelerometer();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::rebootMagnetometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.rebootMagnetometer();
        break;
        case LSM303C:
            lsm303c.rebootMagnetometer();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.enableInterrupt1(axesEvents, threshold, duration, (Sodaq_LSM303AGR::InterruptMode)interruptMode);
        break;
        case LSM303C:
            lsm303c.enableInterrupt1(axesEvents, threshold, duration, (Sodaq_LSM303C::InterruptMode)interruptMode);
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::disableInterrupt1()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.disableInterrupt1();
        break;
        case LSM303C:
            lsm303c.disableInterrupt1();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.enableInterrupt2(axesEvents, threshold, duration, (Sodaq_LSM303AGR::InterruptMode)interruptMode);
        break;
        case LSM303C:
            lsm303c.enableInterrupt2(axesEvents, threshold, duration, (Sodaq_LSM303C::InterruptMode)interruptMode);
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::disableInterrupt2()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.disableInterrupt2();
        break;
        case LSM303C:
            lsm303c.disableInterrupt2();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt)
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.enableMagnetometerInterrupt(magAxesEvents, threshold, highOnInterrupt);
        break;
        case LSM303C:
            lsm303c.enableMagnetometerInterrupt( magAxesEvents, threshold, highOnInterrupt);
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

void Sodaq_LSM303AGR_C::disableMagnetometerInterrupt()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            lsm303agr.disableMagnetometerInterrupt();
        break;
        case LSM303C:
            lsm303c.disableMagnetometerInterrupt();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
}

double Sodaq_LSM303AGR_C::getX()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = lsm303agr.getX();
        break;
        case LSM303C:
            result = lsm303c.getX();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
    return result;
}

double Sodaq_LSM303AGR_C::getY()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = lsm303agr.getY();
        break;
        case LSM303C:
            result = lsm303c.getY();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
    return result;
}

double Sodaq_LSM303AGR_C::getZ()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = lsm303agr.getZ();
        break;
        case LSM303C:
            result = lsm303c.getZ();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
    return result;
}

double Sodaq_LSM303AGR_C::getMagX()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = lsm303agr.getMagX();
        break;
        case LSM303C:
            result = lsm303c.getMagX();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
    return result;
}

double Sodaq_LSM303AGR_C::getMagY()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = lsm303agr.getMagY();
        break;
        case LSM303C:
            result = lsm303c.getMagY();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
    return result;
}

double Sodaq_LSM303AGR_C::getMagZ()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = lsm303agr.getMagZ();
        break;
        case LSM303C:
            result = lsm303c.getMagZ();
        break;
        default:
            ERRORMESSAGE;
        break;
    }
    return result;
}
