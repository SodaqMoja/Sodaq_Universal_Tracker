/*
Copyright (c) 2016-18, SODAQ
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

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <Arduino.h>

typedef void(*VoidCallbackMethodPtr)(void);

struct ConfigParams
{
    uint16_t _header;

    uint16_t _defaultFixInterval;
    uint16_t _alternativeFixInterval;
    uint8_t _alternativeFixFromHours;
    uint8_t _alternativeFixFromMinutes;
    uint8_t _alternativeFixToHours;
    uint8_t _alternativeFixToMinutes;
    uint8_t _gpsFixTimeout;

    uint8_t _accelerationPercentage;
    uint8_t _accelerationDuration;
    uint8_t _onTheMoveFixInterval;
    uint8_t _onTheMoveTimeout;

    uint8_t _isLedEnabled;
    uint8_t _isOtaaEnabled;
    uint8_t _shouldRetryConnectionOnSend;

    char _devAddrOrEUI[16 + 1];
    char _appSKeyOrEUI[32 + 1];
    char _nwSKeyOrAppKey[32 + 1];

    char _attToken[32 + 1];

    char _apn[32 + 1];
    char _forceOperator[32 + 1];
    uint8_t _cid;
    uint8_t _mnoProfile;

    char _apnUser[32 + 1];
    char _apnPassword[32 + 1];

    char _band[36 + 1];
    uint8_t _rxTimeout;

    char _targetIP[50]; // IP or DNS = 50 

    uint16_t _targetPort;

    uint8_t _loraPort;
    uint8_t _isAdrOn;
    uint8_t _isAckOn;
    uint8_t _spreadingFactor;
    uint8_t _powerIndex;

    uint8_t _networkType;

    uint8_t _gpsMinSatelliteCount;
    uint8_t _coordinateUploadCount;
    uint8_t _repeatCount;

    uint8_t _isDebugOn;
    uint8_t _isCayennePayloadEnabled;

    uint16_t _crc16;

public:
    void read();
    void commit(bool forced = false);
    void reset();

    bool execCommand(const char* line);

    uint16_t getDefaultFixInterval() const { return _defaultFixInterval; }
    uint16_t getAlternativeFixInterval() const { return _alternativeFixInterval; }
    uint8_t getAlternativeFixFromHours() const { return _alternativeFixFromHours; }
    uint8_t getAlternativeFixFromMinutes() const { return _alternativeFixFromMinutes; }
    uint32_t getAlternativeFixFrom() const { return _alternativeFixFromHours * 60 * 60 + _alternativeFixFromMinutes * 60; }
    uint8_t getAlternativeFixToHours() const { return _alternativeFixToHours; }
    uint8_t getAlternativeFixToMinutes() const { return _alternativeFixToMinutes; }
    uint32_t getAlternativeFixTo() const { return _alternativeFixToHours * 60 * 60 + _alternativeFixToMinutes * 60; }
    uint8_t getGpsFixTimeout() const { return _gpsFixTimeout; }

    uint8_t getAccelerationPercentage() const { return _accelerationPercentage; }
    uint8_t getAccelerationDuration() const { return _accelerationDuration; }
    uint8_t getOnTheMoveFixInterval() const { return _onTheMoveFixInterval; }
    uint8_t getOnTheMoveTimeout() const { return _onTheMoveTimeout; }

    uint8_t getIsLedEnabled() const { return _isLedEnabled; }
    uint8_t getIsOtaaEnabled() const { return _isOtaaEnabled; }
    uint8_t getShouldRetryConnectionOnSend() const { return _shouldRetryConnectionOnSend; }

    const char* getDevAddrOrEUI() const { return _devAddrOrEUI; }
    const char* getAppSKeyOrEUI() const { return _appSKeyOrEUI; }
    const char* getNwSKeyOrAppKey() const { return _nwSKeyOrAppKey; }

    const char* getApn() const { return _apn; }
    const char* getForceOperator() const { return _forceOperator; }

    const char* getAttToken() const { return _attToken; }

    const char* getApnUser() const { return _apnUser; }
    const char* getApnPassword() const { return _apnPassword; }

    uint8_t getCID() const { return _cid; }
    uint8_t getMnoProfile() const { return _mnoProfile; }

    const char* getBand() const { return _band; }
    uint8_t getRXtimeout() const { return _rxTimeout; }

    const char* getTargetIP() const { return _targetIP; }

    uint16_t getTargetPort() const { return _targetPort; }

    uint8_t getNetworkType() const { return _networkType; }

    uint8_t getLoraPort() const { return _loraPort; }
    uint8_t getIsAdrOn() const { return _isAdrOn; }
    uint8_t getIsAckOn() const { return _isAckOn; }
    uint8_t getSpreadingFactor() const { return _spreadingFactor; }
    uint8_t getPowerIndex() const { return _powerIndex; }

    uint8_t getGpsMinSatelliteCount() const { return _gpsMinSatelliteCount; }
    uint8_t getCoordinateUploadCount() const { return _coordinateUploadCount; }
    uint8_t getRepeatCount() const { return _repeatCount; }

    uint8_t getIsDebugOn() const { return _isDebugOn; }
    uint8_t getIsCayennePayloadEnabled() const { return _isCayennePayloadEnabled; }

    static void showConfig(Stream* stream);
    bool checkConfig(Stream& stream);
    void setConfigResetCallback(VoidCallbackMethodPtr callback);
};

extern ConfigParams params;

#endif
