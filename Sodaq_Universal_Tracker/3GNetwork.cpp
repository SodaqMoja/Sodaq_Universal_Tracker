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

#include "3GNetwork.h"
#include "Config.h"

#define DEBUG

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

// BEGIN 3G DEFINES
#ifndef PIN_SARA_ENABLE
#define SARA_TX_ENABLE -1
#define SARA_ENABLE -1
#endif // !ARDUINO_SODAQ_SARA
// END 3G DEFINES

void Network3G::printIpTuple(uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4)
{
    debugPrint(o1);
    debugPrint(".");
    debugPrint(o2);
    debugPrint(".");
    debugPrint(o3);
    debugPrint(".");
    debugPrintLn(o4);
}

/**
Initializes the 3G module according to the given operation (join or skip).
Returns true if the operation was successful.
*/

bool Network3G::init(Uart & modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join)
{
    _callback = callback;
    _baudRate = sodaq_3gbee.getDefaultBaudrate();
    modemStream.begin(sodaq_3gbee.getDefaultBaudrate());

    if (params.getIsDebugOn()) {
        if (_diagStream) {
            sodaq_3gbee.setDiag(_diagStream); // optional
        }
    }

    delay(500);

    sodaq_3gbee.init(modemStream, -1, SARA_ENABLE, -1);

    if (SARA_TX_ENABLE >= 0) {
        pinMode(SARA_TX_ENABLE, OUTPUT); // maybe needed for other SARA boards
        digitalWrite(SARA_TX_ENABLE, HIGH);
    }

    sodaq_3gbee.setApn(params.getApn(), params.getApnUser(), params.getApnPassword());

    if (join == INIT_JOIN) {
        // trigger connection
        setActive(true);
    }
    else {
        sodaq_3gbee.on();
    }

    bool modemAlive = sodaq_3gbee.isAlive();

    return modemAlive;
}

/**
* Turns the module on or off (and connects/disconnects)
*/

bool Network3G::setActive(bool on)
{
    bool success = false;
    if (on) {
        success = sodaq_3gbee.connect();
    }
    else {
        success = sodaq_3gbee.off();
    }
    return success;
}

uint8_t Network3G::transmit(uint8_t * buffer, uint8_t size, uint32_t rxTimeout)
{
    if(!setActive(true)) {
        return false;
    }

    uint8_t socket = sodaq_3gbee.createSocket(UDP);
    if (sodaq_3gbee.connectSocket(socket, params.getTargetIP(), params.getTargetPort())) {
        debugPrintLn("socket connected");

        sodaq_3gbee.socketSend(socket, buffer, size);

        uint8_t receiveBuffer[128];
        size_t bytesRead = sodaq_3gbee.socketReceive(socket, receiveBuffer, sizeof(receiveBuffer), rxTimeout);

        _callback(receiveBuffer, bytesRead);

        sodaq_3gbee.closeSocket(socket);
        return bytesRead;
    }
    else {
        return 0;
    }
}

void Network3G::loopHandler()
{
    // TODO: add support for retransmission ?
}

void Network3G::sleep()
{
    setActive(false);
}

bool Network3G::getIMEI(char * buffer, size_t size)
{
    return sodaq_3gbee.getIMEI(buffer, size);
}

bool Network3G::getCCID(char* buffer, size_t size)
{
    return sodaq_3gbee.getCCID(buffer, size);
}