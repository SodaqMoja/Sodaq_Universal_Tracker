/*
Copyright (c) 2020, SODAQ
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

#include "Sodaq_N3X.h"
#include "Sodaq_wdt.h"
#include "Config.h"
#include "N3xNetwork.h"

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
static Sodaq_N3X n3x;
static Sodaq_SARA_N310_OnOff saraN310OnOff;
// END NBiot DEFINES

/**
Initializes the n3x module according to the given operation (join or skip).
Returns true if the operation was successful.
*/
bool N3xNetwork::init(Uart & modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join)
{
    _callback = callback;

    if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing N3X...");
    }

    if (_diagStream != _consoleStream || messages != INIT_SHOW_CONSOLE_MESSAGES) {
        debugPrintLn("Initializing N3X...");
    }

    if (params.getIsDebugOn() && _diagStream) {
        n3x.setDiag(_diagStream);
    }

    modemStream.begin(n3x.getDefaultBaudrate());
    n3x.init(&saraN310OnOff, modemStream, params.getCID());

    if (join == INIT_JOIN) {
        setActive(true, false);
    }
    else {
        n3x.on();
        n3x.purgeAllResponsesRead();
    }

    return n3x.isAlive();
}

/**
* Turns the nbiot module on or off (and connects/disconnects)
*/
bool N3xNetwork::setActive(bool on, bool needCheckConnection)
{
    sodaq_wdt_reset();
    bool success = false;

    if (!on || (needCheckConnection && n3x.isConnected())) {
        return true;
    }

    if (!needCheckConnection) {
        return n3x.on();
    }

    success = n3x.connect(params.getApn(), params.getForceOperator(), params.getBand());
    if (!success) {
        n3x.off();
        sodaq_wdt_safe_delay(450);
        n3x.on();
        sodaq_wdt_safe_delay(450);

        // try just one last time
        success = n3x.connect(params.getApn(), params.getForceOperator(), params.getBand());
    }

    return success;
}

uint8_t N3xNetwork::transmit(uint8_t* buffer, uint8_t size, uint32_t rxTimeout)
{
    if(!setActive(true)) {
        return false;
    }

    debugPrintLn("\n\rSending message through UDP");

    int socketID = n3x.socketCreate(16666);

    if (socketID < 0 || socketID >= 7) {
        debugPrintLn("Failed to create socket");
        return false;
    }

    size_t lengthSent = n3x.socketSend(socketID, params.getTargetIP(), params.getTargetPort(), buffer, size);

    // wait for data
    if (n3x.socketWaitForReceive(socketID, rxTimeout)) {
        debugPrintLn("Received UDP response...");

        uint32_t startTime = millis();

        while (n3x.socketHasPendingBytes(socketID) && (millis() - startTime) < 20000) {
            uint8_t data[128];
            int size = n3x.socketReceive(socketID, data, sizeof(data));
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

    n3x.socketClose(socketID);
    debugPrintLn();

    setActive(false);

    return lengthSent;
}

void N3xNetwork::loopHandler() {}

void N3xNetwork::sleep() {}

uint32_t N3xNetwork::getBaudRate()
{
    return n3x.getDefaultBaudrate();
}

bool N3xNetwork::getIMEI(char* buffer, size_t size)
{
    return n3x.getIMEI(buffer, size);
}

bool N3xNetwork::getCCID(char* buffer, size_t size)
{
    return n3x.getCCID(buffer, size);
}

bool N3xNetwork::getIMSI(char* buffer, size_t size)
{
    return n3x.getIMSI(buffer, size);
}

bool N3xNetwork::getModuleVersion(char* buffer, size_t size)
{
    return n3x.getFirmwareRevision(buffer, size);
}
