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

#include "Config.h"
#include "Command.h"
#include "FlashStorage.h"

#define DEFAULT_HEADER 0xBEEF

ConfigParams params;

FlashStorage(flash, ConfigParams);
static bool needsCommit;
static VoidCallbackMethodPtr configResetCallback;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void ConfigParams::read()
{
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16) {
        reset();
    }
}

void ConfigParams::reset()
{
    _defaultFixInterval = 15;
    _alternativeFixInterval = 0;
    _alternativeFixFromHours = 0;
    _alternativeFixFromMinutes = 0;
    _alternativeFixToHours = 0;
    _alternativeFixToMinutes = 0;
    _gpsFixTimeout = 120;

    memset(_devAddrOrEUI, 0x30, sizeof(_devAddrOrEUI) - 1);
    _devAddrOrEUI[sizeof(_devAddrOrEUI) - 1] = '\0';

    memset(_appSKeyOrEUI, 0x30, sizeof(_appSKeyOrEUI) - 1);
    _appSKeyOrEUI[sizeof(_appSKeyOrEUI) - 1] = '\0';

    memset(_nwSKeyOrAppKey, 0x30, sizeof(_nwSKeyOrAppKey) - 1);
    _nwSKeyOrAppKey[sizeof(_nwSKeyOrAppKey) - 1] = '\0';

    memset(_apn, 0x30, sizeof(_apn) - 1);
    _apn[sizeof(_apn) - 1] = '\0';

    memset(_forceOperator, 0x30, sizeof(_forceOperator) - 1);
    _forceOperator[sizeof(_forceOperator) - 1] = '\0';

    memset(_apnUser, 0x30, sizeof(_apnUser) - 1);
    _apnUser[sizeof(_apnUser) - 1] = '\0';

    memset(_apnPassword, 0x30, sizeof(_apnPassword) - 1);
    _apnPassword[sizeof(_apnPassword) - 1] = '\0';

    memset(_band, 0x30, sizeof(_band) - 1);
    _band[sizeof(_band) - 1] = '\0';

    _rxTimeout = 15;

    memcpy(_targetIP, "0.0.0.0", sizeof("0.0.0.0"));
    _targetIP[sizeof(_targetIP) - 1] = '\0';

    _cid = 1;
    _mnoProfile = 1;

    _targetPort = 0;

    _networkType = 0;

    _coordinateUploadCount = 1;
    _repeatCount = 0;

    _accelerationPercentage = 0;
    _accelerationDuration = 0;
    _onTheMoveFixInterval = 1;
    _onTheMoveTimeout = 10;

    _loraPort = 1;
    _isAdrOn = 1;
    _isAckOn = 0;
    _spreadingFactor = 7;
    _powerIndex = 1;
    _gpsMinSatelliteCount = 4;

    _isLedEnabled = 0;
    _isDebugOn = 0;

    _shouldRetryConnectionOnSend = true;

    if (configResetCallback) {
        configResetCallback();
    }

    needsCommit = true;
}

/*
 * Write the configuration parameters to NVM / Dataflash
 */
void ConfigParams::commit(bool forced)
{
    if (!forced && !needsCommit) {
        return;
    }

    _header = DEFAULT_HEADER;
    uint16_t newCrc16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);
    if(_crc16 != newCrc16) {
        _crc16 = newCrc16;
        flash.write(*this);
    }

    needsCommit = false;
}

static const Command args[] = {
    { "GPS                       ", 0,      0,                  Command::show_title, 0 },
    { "Fix Interval (min)        ", "fi=", Command::set_uint16, Command::show_uint16, &params._defaultFixInterval },
    { "Alt. Fix Interval (min)   ", "afi=", Command::set_uint16, Command::show_uint16, &params._alternativeFixInterval },
    { "Alt. Fix From (HH)        ", "affh=", Command::set_uint8, Command::show_uint8, &params._alternativeFixFromHours },
    { "Alt. Fix From (MM)        ", "affm=", Command::set_uint8, Command::show_uint8, &params._alternativeFixFromMinutes },
    { "Alt. Fix To (HH)          ", "afth=", Command::set_uint8, Command::show_uint8, &params._alternativeFixToHours },
    { "Alt. Fix To (MM)          ", "aftm=", Command::set_uint8, Command::show_uint8, &params._alternativeFixToMinutes },
    { "GPS Fix Timeout (sec)     ", "gft=", Command::set_uint8, Command::show_uint8, &params._gpsFixTimeout },
    { "Minimum sat count         ", "sat=", Command::set_uint8, Command::show_uint8, &params._gpsMinSatelliteCount },
    { "Num Coords to Upload      ", "num=", Command::set_uint8, Command::show_uint8, &params._coordinateUploadCount },
    { "On-the-move Functionality ", 0,      0,                  Command::show_title, 0 },
    { "Acceleration% (100% = 8g) ", "acc=", Command::set_uint8, Command::show_uint8, &params._accelerationPercentage },
    { "Acceleration Duration     ", "acd=", Command::set_uint8, Command::show_uint8, &params._accelerationDuration },
    { "Fix Interval (min)        ", "acf=", Command::set_uint8, Command::show_uint8, &params._onTheMoveFixInterval },
    { "Timeout (min)             ", "act=", Command::set_uint8, Command::show_uint8, &params._onTheMoveTimeout },
#if defined(ARDUINO_SODAQ_ONE)
    { "LoRa                      ", 0,      0,                  Command::show_title, 0 },
    { "OTAA Mode (OFF=0 / ON=1)  ", "otaa=", Command::set_uint8, Command::show_uint8, &params._isOtaaEnabled },
    { "Retry conn. (OFF=0 / ON=1)", "retry=", Command::set_uint8, Command::show_uint8, &params._shouldRetryConnectionOnSend },
    { "ADR (OFF=0 / ON=1)        ", "adr=", Command::set_uint8, Command::show_uint8, &params._isAdrOn },
    { "ACK (OFF=0 / ON=1)        ", "ack=", Command::set_uint8, Command::show_uint8, &params._isAckOn },
    { "Spreading Factor          ", "sf=", Command::set_uint8, Command::show_uint8, &params._spreadingFactor },
    { "Output Index              ", "pwr=", Command::set_uint8, Command::show_uint8, &params._powerIndex },
    { "Lora Port                 ", "lprt=", Command::set_uint8, Command::show_uint8, &params._loraPort },
    { "DevAddr / DevEUI          ", "dev=", Command::set_string, Command::show_string, params._devAddrOrEUI, sizeof(params._devAddrOrEUI) },
    { "AppSKey / AppEUI          ", "app=", Command::set_string, Command::show_string, params._appSKeyOrEUI, sizeof(params._appSKeyOrEUI) },
    { "NWSKey / AppKey           ", "key=", Command::set_string, Command::show_string, params._nwSKeyOrAppKey, sizeof(params._nwSKeyOrAppKey) },
    { "Repeat Count              ", "rep=", Command::set_uint8, Command::show_uint8, &params._repeatCount },
#elif defined(ARDUINO_SODAQ_SFF) || defined (ARDUINO_SODAQ_SARA)
    { "Cellular                  ", 0,      0,                  Command::show_title, 0 },
    { "Network Type (N2xx NB-IoT = 2, R4xx NB-IoT = 3, R4xx LTE-M = 4, R412 2G = 5, 2G/3G = 6) ", "ntype=", Command::set_uint8, Command::show_uint8, &params._networkType },
    { "All Things Talk Token     ", "att=", Command::set_string, Command::show_string, params._attToken, sizeof(params._attToken) },
    { "APN                       ", "apn=", Command::set_string, Command::show_string, params._apn, sizeof(params._apn) },
    { "Force Operator            ", "opr=", Command::set_string, Command::show_string, params._forceOperator, sizeof(params._forceOperator) },
    { "CID                       ", "cid=", Command::set_uint8, Command::show_uint8, &params._cid },
    { "MNO Profile               ", "mno=", Command::set_uint8, Command::show_uint8, &params._mnoProfile },
    { "APN user                  ", "apnu=", Command::set_string, Command::show_string, params._apnUser, sizeof(params._apnUser) },
    { "APN password              ", "apnp=", Command::set_string, Command::show_string, params._apnPassword, sizeof(params._apnPassword) },
    { "Band Info                 ", "bnd=", Command::set_string,  Command::show_string, params._band, sizeof(params._band) },
    { "Target IP or DNS          ", "ip=",  Command::set_string, Command::show_string, params._targetIP, sizeof(params._targetIP) },
    { "Target port               ", "prt=", Command::set_uint16, Command::show_uint16, &params._targetPort },
    { "Response Timeout          ", "rxto=", Command::set_uint8, Command::show_uint8, &params._rxTimeout },
#endif
    { "Misc                      ", 0,      0,                  Command::show_title, 0 },
#if defined(ARDUINO_SODAQ_ONE)
    { "Cayenne LPP (OFF=0 / ON=1)", "cay=", Command::set_uint8, Command::show_uint8, &params._isCayennePayloadEnabled },
#endif
    { "Status LED (OFF=0 / ON=1) ", "led=", Command::set_uint8, Command::show_uint8, &params._isLedEnabled },

    { "Debug (OFF=0 / ON=1)      ", "dbg=", Command::set_uint8, Command::show_uint8, &params._isDebugOn }
};

void ConfigParams::showConfig(Stream* stream)
{
    stream->println();
    stream->println("Settings:");
    for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i) {
        const Command* a = &args[i];
        if (a->show_func) {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool ConfigParams::execCommand(const char* line)
{
    bool done = Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
    if (done) {
        needsCommit = true;
    }

    return done;
}

/*
 * Check if all required config parameters are filled in
 */
bool ConfigParams::checkConfig(Stream& stream)
{
    bool fail = false;

    if (_alternativeFixFromHours > 23) {
        stream.println("\n\nERROR: \"Alt. Fix From (HH)\" must not be more than 23");

        fail = true;
    }

    if (_alternativeFixToHours > 23) {
        stream.println("\n\nERROR: \"Alt. Fix To (HH)\" must not be more than 23");

        fail = true;
    }

    if (_alternativeFixFromMinutes > 59) {
        stream.println("\n\nERROR: \"Alt. Fix From (MM)\" must not be more than 59");

        fail = true;
    }

    if (_alternativeFixToMinutes > 59) {
        stream.println("\n\nERROR: \"Alt. Fix To (MM)\" must not be more than 59");

        fail = true;
    }

    if (_alternativeFixToMinutes > 59) {
        stream.println("\n\nERROR: \"Alt. Fix To (MM)\" must not be more than 59");

        fail = true;
    }

    if (_coordinateUploadCount < 1 || _coordinateUploadCount > 4) {
        stream.println("\n\nERROR: \"Num Coords to Upload\" must be between 1 and 4");

        fail = true;
    }

    if (_isOtaaEnabled > 1) {
        stream.println("OTAA Mode must be either 0 or 1");
        fail = true;
    }

    if (_isAdrOn > 1) {
        stream.println("ADR must be either 0 or 1");
        fail = true;
    }

    if (_isAckOn > 1) {
        stream.println("ACK must be either 0 or 1");
        fail = true;
    }

    if (_isDebugOn > 1) {
        stream.println("Debug must be either 0 or 1");
        fail = true;
    }

    if (_gpsFixTimeout > 250) {
        stream.println("GPS fix timeout must not be more than 250 seconds");
        fail = true;
    }

    if (_accelerationPercentage > 100) {
        stream.println("Acceleration% must not be more than 100");
        fail = true;
    }

    if (_networkType == 0) {
        stream.println("Network type should be set");
        fail = true;
    }

    return !fail;
}

void ConfigParams::setConfigResetCallback(VoidCallbackMethodPtr callback)
{
    configResetCallback = callback;
}
