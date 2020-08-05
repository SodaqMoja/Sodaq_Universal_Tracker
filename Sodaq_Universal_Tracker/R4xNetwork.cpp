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

#include "Sodaq_R4X.h"
#include "Sodaq_wdt.h"
#include "Config.h"
#include "R4xNetwork.h"

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

#define MODEM_BAUD 115200

// BEGIN NBiot DEFINES
static Sodaq_R4X r4x;
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;
// END NBiot DEFINES

/**
Initializes the R4X module according to the given operation (join or skip).
Returns true if the operation was successful.
*/
bool R4xNetwork::init(Uart & modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join, const char* urat)
{
    _callback = callback;

    strncpy(_urat, urat, sizeof(_urat) - 1);

    if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing R4X...");
    }

    if (_diagStream != _consoleStream || messages != INIT_SHOW_CONSOLE_MESSAGES) {
        debugPrintLn("Initializing R4X...");
    }

    if (params.getIsDebugOn() && _diagStream) {
        r4x.setDiag(_diagStream);
    }

    r4x.init(&saraR4xxOnOff, modemStream, MODEM_BAUD);

    if (join == INIT_JOIN) {
        setActive(true, false);
    }
    else {
        r4x.on();
        r4x.purgeAllResponsesRead();
    }

    return true;
}

/**
* Turns the nbiot module on or off (and connects/disconnects)
*/
bool R4xNetwork::setActive(bool on, bool needCheckConnection)
{
    sodaq_wdt_reset();
    bool success = false;

    if (!on) {
        r4x.off();
        return true;
    }

    if (needCheckConnection && r4x.isConnected()) {
        return true;
    }

    success = r4x.connect(params.getApn(), _urat, (MNOProfile)params.getMnoProfile(), params.getForceOperator(), params.getBand(), params.getBand());
    if (!success) {
        r4x.off();
        sodaq_wdt_safe_delay(450);
        r4x.on();
        sodaq_wdt_safe_delay(450);

        // try just one last time
        success = r4x.connect(params.getApn(), _urat, (MNOProfile)params.getMnoProfile(), params.getForceOperator(), params.getBand(), params.getBand());
    }
    
    // Turn off PSM and eDRX
    r4x.execCommand("AT+CPSMS=0");
    r4x.execCommand("AT+CEDRXS=0");
    r4x.execCommand("AT+UPSV=0");
    
    return success;
}

uint8_t R4xNetwork::transmit(uint8_t* buffer, uint8_t size, uint32_t rxTimeout)
{
    if(!setActive(true)) {
        return false;
    }

    debugPrintLn("\n\rSending message through UDP");

    int socketID = r4x.socketCreate(16666);

    if (socketID < 0 || socketID >= 7) {
        debugPrintLn("Failed to create socket");
        return false;
    }

    size_t lengthSent = r4x.socketSend(socketID, params.getTargetIP(), params.getTargetPort(), buffer, size);

    // wait for data
    if (r4x.socketWaitForReceive(socketID, rxTimeout)) {
        debugPrintLn("Received UDP response...");

        uint32_t startTime = millis();

        while (r4x.socketHasPendingBytes(socketID) && (millis() - startTime) < 20000) {
            uint8_t data[128];
            int size = r4x.socketReceive(socketID, data, sizeof(data));
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

    r4x.socketClose(socketID);
    debugPrintLn();

    setActive(false);

    return lengthSent;
}

void R4xNetwork::loopHandler() {}

void R4xNetwork::sleep() {}

uint32_t R4xNetwork::getBaudRate()
{
    return MODEM_BAUD;
}

bool R4xNetwork::getIMEI(char* buffer, size_t size)
{
    return r4x.getIMEI(buffer, size);
}

bool R4xNetwork::getCCID(char* buffer, size_t size)
{
    return r4x.getCCID(buffer, size);
}

bool R4xNetwork::getModuleVersion(char* buffer, size_t size)
{
    return r4x.getFirmwareRevision(buffer, size);
}
