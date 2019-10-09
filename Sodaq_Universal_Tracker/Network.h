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

#include <Arduino.h>
#include "DataReceiveCallback.h"
#include "Enums.h"
#include "LoRaHelper.h"
#include "LoraNetwork.h"
#include "N2xNetwork.h"
#include "R4xNetwork.h"
#include "3GNetwork.h"

class Network {
public:
    enum NetworkType {
        NETWORK_TYPE_NOTYPE = 0,
        NETWORK_TYPE_LORA,
        NETWORK_TYPE_NBIOT_N2,
        NETWORK_TYPE_NBIOT_R4,
        NETWORK_TYPE_LTEM_R4,
        NETWORK_TYPE_2G_R4,
        NETWORK_TYPE_2G_3G,
    };

    bool init(Uart& modemStream, DataReceiveCallback callback, uint32_t(*getNow)(), InitConsoleMessages messages, InitJoin join);

    uint8_t transmit(uint8_t* buffer, uint8_t size, uint32_t rxTimeout);

    void loopHandler();

    void sleep();

    bool setActive(bool on);

    void setNetworkType(NetworkType type);

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream& stream) { _diagStream = &stream; }
    void setDiag(Stream* stream) { _diagStream = stream; }
    void setConsoleStream(Stream& stream) { _consoleStream = &stream; }
    LoraNetwork& getLoraNetwork();
    const char* getIMEI();
    const char* getCCID();
    const char* getModuleVersion();

    uint32_t getBaudRate();

private:
    NetworkType _networkType = NETWORK_TYPE_NOTYPE;
    // The (optional) stream to show debug information.
    Stream* _diagStream;
    Stream* _consoleStream;
};
