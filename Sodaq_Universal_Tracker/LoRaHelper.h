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

#ifndef _LORAHELPER_h
#define _LORAHELPER_h

#include "Arduino.h"
#include "Sodaq_RN2483.h"

#define LORA_MAX_PACKET_LENGTH 250
#define LORA_MAX_RETRANSMISSIONS 3
#define LORA_MIN_RETRASMISSION_BACKOFF_SEC 10
#define LORA_RETRANSMISSION_ATTEMPTS_REPLENISH_AFTER_TRANSMIT_SEC 10 * 60

class LoRaHelper
{
public:
    LoRaHelper();

    void setDiag(Stream& stream) { _diagStream = &stream; };
    void init(Sodaq_RN2483& rn2483, uint32_t(*getNow)());
    void setKeys(const char* devAddrOrEUI, const char* appSKeyOrEUI, const char* nwSKeyOrAppKey) {
        _devAddrOrEUI = devAddrOrEUI;
        _appSKeyOrEUI = appSKeyOrEUI;
        _nwSKeyOrAppKey = nwSKeyOrAppKey;
    };
    void setOtaaOn(bool on) { _isOtaaOn = on; };
    bool isOtaaOn() { return _isOtaaOn; };

    void setAdrOn(bool on) { _isAdrOn = on; };
    bool isAdrOn() { return _isAdrOn; };

    void setAckOn(bool on) { _isAckOn = on; };
    bool isAckOn() { return _isAckOn; };

    void setReconnectOnTransmissionOn(bool on) { _isReconnectOnTransmissionOn = on; };
    bool isReconnectOnTransmissionOn() { return _isReconnectOnTransmissionOn; };

    void setDefaultLoRaPort(uint8_t port) { _defaultLoRaPort = port; };
    uint8_t getDefaultLoRaPort() { return _defaultLoRaPort; }

    void setRepeatTransmissionCount(uint8_t count) { _repeatTransmissionCount = count; };
    uint8_t getRepeatTransmissionCount() { return _repeatTransmissionCount; };

    void setSpreadingFactor(uint8_t spreadingFactor) { _spreadingFactor = spreadingFactor; };
    void setPowerIndex(uint8_t powerIndex) { _powerIndex = powerIndex; };
    void setActive(bool on);
    bool isInitialized() { return _isInitialized; };
    void setInitialized(bool isInitialized) { _isInitialized = isInitialized; }
    uint8_t getHWEUI(uint8_t* hweui, uint8_t size);
    bool join();
    uint8_t transmit(uint8_t* buffer, uint8_t size, int16_t overrideLoRaPort = -1);
    void extendSleep();
    void loopHandler();
private:
    Stream* _diagStream;
    Sodaq_RN2483* _rn2483;
    const char* _devAddrOrEUI;
    const char* _appSKeyOrEUI;
    const char* _nwSKeyOrAppKey;
    bool _isOtaaOn;
    bool _isAdrOn;
    bool _isAckOn;
    bool _isReconnectOnTransmissionOn;
    uint8_t _defaultLoRaPort;
    uint8_t _repeatTransmissionCount;
    uint8_t _spreadingFactor;
    uint8_t _powerIndex;
    bool _isInitialized;
    int8_t _transmissionAttemptsRemaining;
    bool _isRetransmissionPending;
    uint8_t _retransmissionPacketBuffer[LORA_MAX_PACKET_LENGTH];
    uint8_t _retransmissionPacketSize;
    int16_t _retransmissionOverrideLoRaPort;
    uint32_t _lastTransmissionAttemptTimestamp;
    uint32_t (*_getNow)();

    bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize);
    bool joinAbp();
    bool joinOtaa();
    void retransmissionUpdateOnSuccess();
    void retransmissionUpdateOnFailure();
};

extern LoRaHelper LoRa;

#endif
