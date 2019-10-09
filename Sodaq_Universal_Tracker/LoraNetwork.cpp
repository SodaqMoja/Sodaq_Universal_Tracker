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

#include "LoraNetwork.h"
#include "Config.h"

// #define DEBUG

#ifdef DEBUG
#define debugPrintLn(...) { if (this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

#ifndef consolePrint
#define consolePrint(...) { if (this->_consoleStream) this->_consoleStream->println(__VA_ARGS__); }
#define consolePrintln(...) { if (this->_consoleStream) this->_consoleStream->println(__VA_ARGS__); }
#endif


// BEGIN LORA DEFINES
#define LORA_PORT 1
#define LORA_MAX_RETRIES 3

static uint8_t loraHWEui[8];
static bool isLoraHWEuiInitialized;

#ifndef LORA_RESET
#define LORA_RESET -1
#endif
// END LORA DEFINES

/**
Initializes the lora module according to the given operation (join or skip).
Returns true if the operation was successful.
*/

bool LoraNetwork::init(Uart & modemStream, DataReceiveCallback callback, uint32_t(*getNow)(), InitConsoleMessages messages, InitJoin join)
{
    this->modemStream = &modemStream;
    debugPrintLn("Initializing LoRa...");

    if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing LoRa...");
    }

    if (params.getIsDebugOn()) {
        if (_diagStream) {
            LoRaBee.setDiag(*_diagStream);
            LoRa.setDiag(*_diagStream);
        }
    }

    bool result;

    modemStream.begin(LoRaBee.getDefaultBaudRate());

    LoRaBee.setReceiveCallback(callback);
    LoRa.init(LoRaBee, getNow);

    // set keys and parameters
    LoRa.setKeys(params.getDevAddrOrEUI(), params.getAppSKeyOrEUI(), params.getNwSKeyOrAppKey());
    LoRa.setOtaaOn(params.getIsOtaaEnabled());
    LoRa.setAdrOn(params.getIsAdrOn());
    LoRa.setAckOn(params.getIsAckOn());
    LoRa.setReconnectOnTransmissionOn(params.getShouldRetryConnectionOnSend());
    LoRa.setDefaultLoRaPort(params.getLoraPort());
    LoRa.setRepeatTransmissionCount(params.getRepeatCount());
    LoRa.setSpreadingFactor(params.getSpreadingFactor());
    LoRa.setPowerIndex(params.getPowerIndex());

    if (LoRaBee.initResume(modemStream, LORA_RESET)) {
        LoRa.setInitialized(true);
        result = true;
    }
    else {
        result = LoRaBee.init(modemStream, LORA_RESET, true, true);

        if (join == INIT_JOIN) {
            result = LoRa.join();

            if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
                if (result) {
                    consolePrintln("LoRa initialized.");
                }
                else {
                    consolePrintln("LoRa initialization failed!");
                }
            }
        }
    }

    LoRa.setActive(false); // make sure it is off
    return result;
}

uint8_t* LoraNetwork::getHWEUI(Uart & modemStream, DataReceiveCallback callback, uint32_t(*getNow)())
{
    // only read the HWEUI once
    if (!isLoraHWEuiInitialized) {
        if (init(modemStream, callback, getNow, INIT_SKIP_CONSOLE_MESSAGES, INIT_SKIP_JOIN)) {
            sodaq_wdt_safe_delay(10);
            uint8_t len = LoRa.getHWEUI(loraHWEui, sizeof(loraHWEui));

            if (len == sizeof(loraHWEui)) {
                isLoraHWEuiInitialized = true;
            }
        }
    }
    return loraHWEui;
}

bool LoraNetwork::resetLora(Uart& modemStream)
{
    modemStream.begin(LoRaBee.getDefaultBaudRate());
    return LoRaBee.init(modemStream, LORA_RESET, false, true) && LoRaBee.saveState();
}

void LoraNetwork::setDevAddrOrEUItoHWEUI(Uart & modemStream, DataReceiveCallback callback, uint32_t(*getNow)())
{
    getHWEUI(modemStream, callback, getNow);

    if (isLoraHWEuiInitialized) {
        for (uint8_t i = 0; i < sizeof(loraHWEui); i++) {
            params._devAddrOrEUI[i * 2] = NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(loraHWEui[i]));
            params._devAddrOrEUI[i * 2 + 1] = NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(loraHWEui[i]));
        }
    }

}

void LoraNetwork::sleep()
{
    // stream is flushed before sleep is called
}

/**
* Turns the LoRa module on or off (wake up or sleep)
*/

bool LoraNetwork::setActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        LoRaBee.wakeUp();
    }
    else {
        LoRaBee.sleep();
    }
    
    return true;
}
