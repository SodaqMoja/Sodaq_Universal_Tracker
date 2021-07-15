#ifndef SODAQ_LSM303AGR_C_H_
#define SODAQ_LSM303AGR_C_H_

#include <stdint.h>
#include <Wire.h>
#include "Sodaq_LSM303AGR.h"
#include "Sodaq_LSM303C.h"

#define Sodaq_LSM303C_ACCEL_ADDRESS 0b0011101
#define SODAQ_LSM303C_MAG_ADDRESS 0b0011110

#define Sodaq_LSM303AGR_ACCEL_ADDRESS 0b0011001
#define SODAQ_LSM303AGR_MAG_ADDRESS 0b0011110

#define WHOAMI_ID_A_LSM303C 0b01000001
#define WHOAMI_ID_M_LSM303C 0b00111101

class Sodaq_LSM303AGR_C
{
public:
    enum AxesEvents
    {
        ZHigh = 0b00100000,
        ZLow = 0b00010000,
        YHigh = 0b00001000,
        YLow = 0b00000100,
        XHigh = 0b00000010,
        XLow = 0b00000001
    };
    
    enum InterruptMode
    {
        OrCombination = 0b00000000,
        MovementRecognition = 0b01000000,
        AndCombination = 0b10000000,
        PositionRecognition = 0b11000000
    };

    enum LSM303Variant
    {
        LSM303_Undefined,
        LSM303AGR,
        LSM303C
    };

    Sodaq_LSM303AGR_C(TwoWire &wire = Wire);

    void Init();

    int8_t getTemperature();
    void enableAccelerometer();
    void disableAccelerometer();
    void rebootAccelerometer();

    void enableMagnetometer();
    void disableMagnetometer();
    void rebootMagnetometer();

    void enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode);
    void disableInterrupt1();
    void enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode);
    void disableInterrupt2();
    void enableMagnetometerInterrupt(uint8_t axesEvents, double threshold, bool highOnInterrupt = true);
    void disableMagnetometerInterrupt();

    double getX();
    double getY();
    double getZ();

    double getMagX();
    double getMagY();
    double getMagZ();

protected:
    TwoWire &_wire;
    LSM303Variant _lsm303Variant = LSM303_Undefined;
};

#endif
