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

#ifndef SODAQ_LSM303_H_
#define SODAQ_LSM303_H_

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

class Sodaq_LSM303 {
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

    Sodaq_LSM303AGR     lsm303agr;
    Sodaq_LSM303C       lsm303c;


    void Init(Stream& stream);

    // Sets the optional "Diagnostics and Debug" stream.
    Sodaq_LSM303(TwoWire& wire = Wire);
    void setDiag(Stream& stream) { _diagStream = &stream; }
    void setDiag(Stream* stream) { _diagStream = stream; }

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

    // The (optional) stream to show debug information.
    Stream* _diagStream;
};

#endif
