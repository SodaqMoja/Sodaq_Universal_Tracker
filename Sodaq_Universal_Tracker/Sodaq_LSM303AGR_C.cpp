#include <Arduino.h>
#include <math.h>
#include "Sodaq_LSM303AGR_C.h"

Sodaq_LSM303AGR     Sodaq_LSM303AGR;
Sodaq_LSM303C       Sodaq_LSM303C;

Sodaq_LSM303AGR_C::Sodaq_LSM303AGR_C(TwoWire &wire) : _wire(wire)
{
}

void Sodaq_LSM303AGR_C::Init()
{
    Wire.beginTransmission(Sodaq_LSM303C_ACCEL_ADDRESS);
    if(Wire.endTransmission() == 0)
    {
        _lsm303Variant = LSM303C;
        SerialUSB.println("FOUND LSM303C");
    }
    else
    {
        Wire.beginTransmission(Sodaq_LSM303AGR_ACCEL_ADDRESS);
        if(Wire.endTransmission() == 0)
        {
            _lsm303Variant = LSM303AGR;
            SerialUSB.println("FOUND LSM303AGR");
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
            temp = Sodaq_LSM303AGR.getTemperature();
        break;
        case LSM303C:
            temp = Sodaq_LSM303C.getTemperature();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }

    return temp;
}

void Sodaq_LSM303AGR_C::enableAccelerometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.enableAccelerometer();
        break;
        case LSM303C:
            Sodaq_LSM303C.enableAccelerometer();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::enableMagnetometer()
{
   switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.enableMagnetometer();
        break;
        case LSM303C:
            Sodaq_LSM303C.enableMagnetometer();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::disableAccelerometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.disableAccelerometer();
        break;
        case LSM303C:
            Sodaq_LSM303C.disableAccelerometer();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::disableMagnetometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.disableMagnetometer();
        break;
        case LSM303C:
            Sodaq_LSM303C.disableMagnetometer();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::rebootAccelerometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.rebootAccelerometer();
        break;
        case LSM303C:
            Sodaq_LSM303C.rebootAccelerometer();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::rebootMagnetometer()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.rebootMagnetometer();
        break;
        case LSM303C:
            Sodaq_LSM303C.rebootMagnetometer();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.enableInterrupt1(axesEvents, threshold, duration, (Sodaq_LSM303AGR::InterruptMode)interruptMode);
        break;
        case LSM303C:
            Sodaq_LSM303C.enableInterrupt1(axesEvents, threshold, duration, (Sodaq_LSM303C::InterruptMode)interruptMode);
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::disableInterrupt1()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.disableInterrupt1();
        break;
        case LSM303C:
            Sodaq_LSM303C.disableInterrupt1();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.enableInterrupt2(axesEvents, threshold, duration, (Sodaq_LSM303AGR::InterruptMode)interruptMode);
        break;
        case LSM303C:
            Sodaq_LSM303C.enableInterrupt2(axesEvents, threshold, duration, (Sodaq_LSM303C::InterruptMode)interruptMode);
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::disableInterrupt2()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.disableInterrupt2();
        break;
        case LSM303C:
            Sodaq_LSM303C.disableInterrupt2();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt)
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.enableMagnetometerInterrupt(magAxesEvents, threshold, highOnInterrupt);
        break;
        case LSM303C:
            Sodaq_LSM303C.enableMagnetometerInterrupt( magAxesEvents, threshold, highOnInterrupt);
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

void Sodaq_LSM303AGR_C::disableMagnetometerInterrupt()
{
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            Sodaq_LSM303AGR.disableMagnetometerInterrupt();
        break;
        case LSM303C:
            Sodaq_LSM303C.disableMagnetometerInterrupt();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
}

double Sodaq_LSM303AGR_C::getX()
{
    double result;
    switch(_lsm303Variant)
    {
        case LSM303AGR:
            result = Sodaq_LSM303AGR.getX();
        break;
        case LSM303C:
            result = Sodaq_LSM303C.getX();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
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
            result = Sodaq_LSM303AGR.getY();
        break;
        case LSM303C:
            result = Sodaq_LSM303C.getY();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
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
            result = Sodaq_LSM303AGR.getZ();
        break;
        case LSM303C:
            result = Sodaq_LSM303C.getZ();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
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
            result = Sodaq_LSM303AGR.getMagX();
        break;
        case LSM303C:
            result = Sodaq_LSM303C.getMagX();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
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
            result = Sodaq_LSM303AGR.getMagY();
        break;
        case LSM303C:
            result = Sodaq_LSM303C.getMagY();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
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
            result = Sodaq_LSM303AGR.getMagZ();
        break;
        case LSM303C:
            result = Sodaq_LSM303C.getMagZ();
        break;
        default:
            SerialUSB.println("Unsupported accelerometer");
        break;
    }
    return result;
}
