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

#pragma once

#include "DataReceiveCallback.h"
#include "Sodaq_wdt.h"
#include "HexUtils.h"
#include "Enums.h"

#include "Sodaq_3Gbee.h"


class Network3G {
public:
    void printIpTuple(uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4);

    /**
    Initializes the lora module according to the given operation (join or skip).
    Returns true if the operation was successful.
    */
    bool init(Uart& modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join);

    /**
    * Turns the nbiot module on or off (and connects/disconnects)
    */
    void setActive(bool on);

    uint8_t transmit(uint8_t* buffer, uint8_t size);

    void loopHandler();

    void sleep();

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream& stream) { _diagStream = &stream; }
    void setDiag(Stream* stream) { _diagStream = stream; }
    void setConsoleStream(Stream& stream) { _consoleStream = &stream; }
    void setConsoleStream(Stream* stream) { _consoleStream = stream; }

    uint32_t getBaudRate() { return _baudRate; }
    bool getIMEI(char* buffer, size_t size);
private:
    DataReceiveCallback _callback;
    // The (optional) stream to show debug information.
    Stream* _diagStream;
    Stream* _consoleStream;
    uint32_t _baudRate;
};
