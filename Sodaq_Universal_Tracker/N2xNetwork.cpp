/*
Copyright (c) 2019, SODAQ
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

#include "Sodaq_N2X.h"
#include "Sodaq_wdt.h"
#include "Config.h"
#include "N2xNetwork.h"

#define DEBUG

#ifdef DEBUG
#define debugPrintLn(...) { if (_diagStream) _diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (_diagStream) _diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

#ifndef consolePrint
#define consolePrint(...) { if (_consoleStream) _consoleStream->println(__VA_ARGS__); }
#define consolePrintln(...) { if (_consoleStream) _consoleStream->println(__VA_ARGS__); }
#endif

// BEGIN NBiot DEFINES
static Sodaq_N2X n2x;
static Sodaq_SARA_N211_OnOff saraN211OnOff;
// END NBiot DEFINES

/**
Initializes the N2X module according to the given operation (join or skip).
Returns true if the operation was successful.
*/
bool N2xNetwork::init(Uart & modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join)
{
    _callback = callback;

    if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing N2X...");
    }

    if (_diagStream != _consoleStream || messages != INIT_SHOW_CONSOLE_MESSAGES) {
        debugPrintLn("Initializing N2X...");
    }

    if (params.getIsDebugOn() && _diagStream) {
        n2x.setDiag(_diagStream);
    }

    modemStream.begin(n2x.getDefaultBaudrate());
    n2x.init(&saraN211OnOff, modemStream, params.getCID());

    if (join == INIT_JOIN) {
        setActive(true, false);
    }
    else {
        n2x.on();
        n2x.purgeAllResponsesRead();
    }

    return n2x.isAlive();
}

/**
* Turns the nbiot module on or off (and connects/disconnects)
*/
bool N2xNetwork::setActive(bool on, bool needCheckConnection)
{
    sodaq_wdt_reset();
    bool success = false;

    if (!on || (needCheckConnection && n2x.isConnected())) {
        return true;
    }

    success = n2x.connect(params._apn, NULL, params._forceOperator, params._band);
    if (!success) {
        n2x.off();
        sodaq_wdt_safe_delay(450);
        n2x.on();
        sodaq_wdt_safe_delay(450);

        // try just one last time
        success = n2x.connect(params._apn, NULL, params._forceOperator, params._band);
    }
    
    return success;
}

uint8_t N2xNetwork::transmit(uint8_t* buffer, uint8_t size, uint32_t rxTimeout)
{
    if(!setActive(true)) {
        return false;
    }

    debugPrintLn("\n\rSending message through UDP");

    int socketID = n2x.socketCreate(16666);

    if (socketID < 0 || socketID >= 7) {
        debugPrintLn("Failed to create socket");
        return false;
    }

    size_t lengthSent = n2x.socketSend(socketID, params.getTargetIP(), params.getTargetPort(), buffer, size);

    // wait for data
    if (n2x.socketWaitForReceive(socketID, rxTimeout)) {
        debugPrintLn("Received UDP response...");

        uint32_t startTime = millis();

        while (n2x.socketHasPendingBytes(socketID) && (millis() - startTime) < 20000) {
            uint8_t data[128];
            int size = n2x.socketReceive(socketID, data, sizeof(data));
            if (size) {
                _callback(data, size);
                startTime = millis();
            }
            else {
                debugPrintLn("Receive failed!");
            }
        }
    }
    else {
        debugPrintLn("Timed-out waiting for UDP Response!");
    }

    n2x.socketClose(socketID);
    debugPrintLn();

    setActive(false);

    return lengthSent;
}

void N2xNetwork::loopHandler() {}

void N2xNetwork::sleep() {}

uint32_t N2xNetwork::getBaudRate()
{
    return n2x.getDefaultBaudrate();
}

bool N2xNetwork::getIMEI(char* buffer, size_t size)
{
    return n2x.getIMEI(buffer, size);
}

bool N2xNetwork::getCCID(char* buffer, size_t size)
{
    return n2x.getCCID(buffer, size);
}

bool N2xNetwork::getModuleVersion(char* buffer, size_t size)
{
    return n2x.getFirmwareRevision(buffer, size);
}
