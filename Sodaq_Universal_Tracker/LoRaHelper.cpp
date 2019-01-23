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

#include "LoRaHelper.h"
#include "Sodaq_wdt.h"

#define LORA_MAX_RETRIES 3

// version of "hex to bin" macro that supports both lower and upper case
#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'a') ? (c - 'a' + 0x0A) : ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0')))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define DEBUG

#ifdef DEBUG
#define debugPrintln(...) { if (this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintln(...)
#define debugPrint(...)
#endif

LoRaHelper LoRa;

LoRaHelper::LoRaHelper() :
    _diagStream(0),
    _rn2483(0),
    _devAddrOrEUI(0),
    _appSKeyOrEUI(0),
    _nwSKeyOrAppKey(0),
    _isOtaaOn(0),
    _isAdrOn(0),
    _isAckOn(0),
    _isReconnectOnTransmissionOn(1),
    _defaultLoRaPort(0),
    _repeatTransmissionCount(0),
    _spreadingFactor(7),
    _powerIndex(1),
    _isInitialized(0),
    _transmissionAttemptsRemaining(LORA_MAX_RETRANSMISSIONS),
    _isRetransmissionPending(0),
    _retransmissionPacketSize(0),
    _retransmissionOverrideLoRaPort(-1),
    _lastTransmissionAttemptTimestamp(0),
    _getNow(0)
{
}

void LoRaHelper::init(Sodaq_RN2483& rn2483, uint32_t(*getNow)())
{
    _rn2483 = &rn2483;
    _getNow = getNow;
};

/**
* Converts the given hex array and returns true if it is valid hex and non-zero.
* "hex" is assumed to be 2*resultSize bytes.
*/
bool LoRaHelper::convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize)
{
    bool foundNonZero = false;

    uint16_t inputIndex = 0;
    uint16_t outputIndex = 0;

    // stop at the first string termination char, or if output buffer is over
    while (outputIndex < resultSize && hex[inputIndex] != 0 && hex[inputIndex + 1] != 0) {
        if (!isxdigit(hex[inputIndex]) || !isxdigit(hex[inputIndex + 1])) {
            return false;
        }

        result[outputIndex] = HEX_PAIR_TO_BYTE(hex[inputIndex], hex[inputIndex + 1]);

        if (result[outputIndex] > 0) {
            foundNonZero = true;
        }

        inputIndex += 2;
        outputIndex++;
    }

    return foundNonZero;
}

bool LoRaHelper::joinOtaa()
{
    uint8_t devEui[8];
    uint8_t appEui[8];
    uint8_t appKey[16];

    bool allParametersValid = convertAndCheckHexArray((uint8_t*)devEui, _devAddrOrEUI, sizeof(devEui))
        && convertAndCheckHexArray((uint8_t*)appEui, _appSKeyOrEUI, sizeof(appEui))
        && convertAndCheckHexArray((uint8_t*)appKey, _nwSKeyOrAppKey, sizeof(appKey));

    // check the parameters first
    if (!allParametersValid) {
        debugPrintln("The parameters for LoRa are not valid. LoRa cannot be enabled.");
        return false;
    }

    return _rn2483->initOTA(devEui, appEui, appKey, _isAdrOn);
}

void LoRaHelper::retransmissionUpdateOnSuccess()
{
    _isRetransmissionPending = false;
    _transmissionAttemptsRemaining = LORA_MAX_RETRANSMISSIONS;
}

void LoRaHelper::retransmissionUpdateOnFailure()
{
    _isRetransmissionPending = true;
    _transmissionAttemptsRemaining--;

    // disable the retransmission mechanism until next counter replenishment
    if (_transmissionAttemptsRemaining == 0) {
        _isRetransmissionPending = false;
    }
}

bool LoRaHelper::joinAbp()
{
    uint8_t devAddr[4];
    uint8_t appSKey[16];
    uint8_t nwkSKey[16];

    bool allParametersValid = convertAndCheckHexArray((uint8_t*)devAddr, _devAddrOrEUI, sizeof(devAddr))
        && convertAndCheckHexArray((uint8_t*)appSKey, _appSKeyOrEUI, sizeof(appSKey))
        && convertAndCheckHexArray((uint8_t*)nwkSKey, _nwSKeyOrAppKey, sizeof(nwkSKey));

    // check the parameters first
    if (!allParametersValid) {
        debugPrintln("The parameters for LoRa are not valid. LoRa cannot be enabled.");
        return false;
    }

    return _rn2483->initABP(devAddr, appSKey, nwkSKey, _isAdrOn);
}

/**
* Sends the current sendBuffer through lora (if enabled).
* Repeats the transmitions according to getRepeatTransmissionCount().
*/
uint8_t LoRaHelper::transmit(uint8_t* buffer, uint8_t size, int16_t overrideLoRaPort)
{
    _lastTransmissionAttemptTimestamp = _getNow();

    uint8_t result = InternalError; // TODO

    if (!isInitialized()) {
        // check for a retry
        if (_isReconnectOnTransmissionOn) {
            // check if join is sucessfull before continuing
            if (!join()) {
                debugPrintln("Failed to reconnect! Hardware-resetting the modem and scheduling a retransmission (if applicable).");
                _rn2483->hardwareReset();
                retransmissionUpdateOnFailure();

                return result;
            }
        }
        else {
            debugPrintln("LoRa is not initialized and \"Reconnect-On-Transmission\" is not enabled. Giving up.");

            return result;
        }
    }

    // cache the parmeters in case a re-transmit is needed
    // but only if the passed buffer is not the cache buffer
    if (buffer != this->_retransmissionPacketBuffer) {
        memcpy(this->_retransmissionPacketBuffer, buffer, size);
        _retransmissionPacketSize = size;
        _retransmissionOverrideLoRaPort = overrideLoRaPort;
    }

    setActive(true);
    uint16_t port = overrideLoRaPort > -1 ? overrideLoRaPort : _defaultLoRaPort;

    for (uint8_t i = 0; i < 1 + getRepeatTransmissionCount(); i++) {

        if (_isAckOn) {
            result = _rn2483->sendReqAck(port, buffer, size, LORA_MAX_RETRIES);
        }
        else {
            result = _rn2483->send(port, buffer, size);
        }

        switch (result)
        {
        case NoError: {
            debugPrintln("Data transmitted successfully.");

            retransmissionUpdateOnSuccess();

            break;
        }
        case NoResponse:
        case Timeout:
        case InternalError:
        case NetworkFatalError:
        case Busy:
        case NotConnected: {
            debugPrint("There was an error while transmitting through LoRaWAN: (");
            debugPrint(result);
            debugPrintln("). Restarting module and scheduling a retransmission.");

            _isInitialized = false;
            _rn2483->hardwareReset();
            sodaq_wdt_safe_delay(250);
            retransmissionUpdateOnFailure();

            break;
        }

        case NoAcknowledgment: {
            if (_isAckOn) {
                debugPrint("There was an error while transmitting through LoRaWAN: (");
                debugPrint(result);
                debugPrintln("). Scheduling a retransmission.");

                retransmissionUpdateOnFailure();
            }
            else {
                debugPrintln("The LoRa module reported a missing ack but no ack was requested. Dismissing the report.");
            }

            break;
        }

        case PayloadSizeError: {
            debugPrintln("Received PayloadSizeError from the module.");

            break;
        }

        default:
            break;
        }
    }

    setActive(false);

    return result;
}

uint8_t LoRaHelper::getHWEUI(uint8_t* hweui, uint8_t size)
{
    setActive(true);
    uint8_t len = _rn2483->getHWEUI(hweui, size);
    setActive(false);

    return len;
}

bool LoRaHelper::join()
{
    debugPrintln("Trying to join...");

    _rn2483->wakeUp();

    bool result;
    if (_isOtaaOn) {
        result = joinOtaa();
    }
    else {
        result = joinAbp();
    }

    if (result) {
        _isInitialized = true;

        if (!_isAdrOn) {
            _rn2483->setSpreadingFactor(_spreadingFactor);
        }

        _rn2483->setPowerIndex(_powerIndex);
    }

    return result;
}

void LoRaHelper::extendSleep()
{
    setActive(true);
    sodaq_wdt_safe_delay(80);
    setActive(false);
}

void LoRaHelper::loopHandler()
{
    if (_isRetransmissionPending
            && (_transmissionAttemptsRemaining > 0)
            && (_getNow() - _lastTransmissionAttemptTimestamp > LORA_MIN_RETRASMISSION_BACKOFF_SEC)) {
        debugPrintln("[LoRaHelper] LoopHandler starting a retransmission.");
        debugPrint("[LoRaHelper] _transmissionAttemptsRemaining = ");
        debugPrintln(_transmissionAttemptsRemaining);
        transmit(_retransmissionPacketBuffer, _retransmissionPacketSize, _retransmissionOverrideLoRaPort);
    }

    // check if it is time for a periodic replenishment of remaining retransmission attempts
    // (otherwise a successful transmission replenishes it anyway)
    if ((_transmissionAttemptsRemaining == 0)
            && (_getNow() - _lastTransmissionAttemptTimestamp > LORA_RETRANSMISSION_ATTEMPTS_REPLENISH_AFTER_TRANSMIT_SEC)){
        debugPrintln("[LoRaHelper] Replenishing attempts counter.");
        retransmissionUpdateOnSuccess();
    }
}

/**
* Turns the LoRa module on or off (wake up or sleep)
*/
void LoRaHelper::setActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        _rn2483->wakeUp();
    }
    else {
        _rn2483->sleep();
    }
}
