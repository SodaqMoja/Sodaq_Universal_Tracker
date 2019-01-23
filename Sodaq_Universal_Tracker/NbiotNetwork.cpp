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

#include "NbiotNetwork.h"

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

// BEGIN NBiot DEFINES
Sodaq_nbIOT nbiot;
// END NBiot DEFINES

#ifndef PIN_SARA_ENABLE
#define SARA_TX_ENABLE 0
#define SARA_RESET 0
#define SARA_ENABLE 0
#define MODEM_ON_OFF_PIN 0
#define SARA_R4XX_TOGGLE 0
#endif // !ARDUINO_SODAQ_SARA

/**
Initializes the NB-IoT module according to the given operation (join or skip).
Returns true if the operation was successful.
*/

bool NbiotNetwork::init(Uart & modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join, bool useR4xx)
{
    _callback = callback;
    debugPrintLn("Initializing NBiot...");

    if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing NBiot...");
    }

    if (params.getIsDebugOn()) {
        if (_diagStream) {
            nbiot.setDiag(_diagStream);
        }
    }

    debugPrintLn("Init nbiot...");
    if (useR4xx) {
        _baudRate = nbiot.getSaraR4Baudrate();
        modemStream.begin(nbiot.getSaraR4Baudrate());
        nbiot.init(modemStream, SARA_ENABLE, -1, SARA_R4XX_TOGGLE, params.getCID());
    }
    else {
        _baudRate = nbiot.getSaraN2Baudrate();
        modemStream.begin(nbiot.getSaraN2Baudrate());
        nbiot.init(modemStream, SARA_ENABLE, -1, -1, params.getCID());
    }
    _useR4xx = useR4xx;


    pinMode(SARA_TX_ENABLE, OUTPUT); // may be needed for other SARA boards
    digitalWrite(SARA_TX_ENABLE, HIGH);

    // the following is on by default in the nbiot library version > v1.4.1
    // nbiot.overrideNconfigParam("CR_0354_0338_SCRAMBLING", true);

    if (join == INIT_JOIN) {
        // trigger connection
        setActive(true);
    }
    else {
        nbiot.on();
    }

    bool modemAlive = nbiot.isAlive();

    return modemAlive;
}

/**
* Turns the nbiot module on or off (and connects/disconnects)
*/

void NbiotNetwork::setActive(bool on)
{
    sodaq_wdt_reset();
    // TODO Fix reset mechanism
    if (on) {
        if (!nbiot.isConnected()) {
            if (!nbiot.connect(params._apn, params._cdp, params._forceOperator, params._band)) {
                nbiot.off();
                sodaq_wdt_safe_delay(450);
                nbiot.on();
                sodaq_wdt_safe_delay(450);

                // try just one last time
                nbiot.connect(params._apn, params._cdp, params._forceOperator, params._band);
            }
        }
    }
    else {
        if (_useR4xx) {
            nbiot.off();
        }
    }
}

uint8_t NbiotNetwork::transmit(uint8_t * buffer, uint8_t size)
{
    setActive(true);

    debugPrintLn();
    debugPrintLn("Sending message through UDP");

    int localPort = 16666;
    int socketID = nbiot.createSocket(localPort);

    if (socketID >= 7 || socketID < 0) {
        debugPrintLn("Failed to create socket");
        return false;
    }

    size_t lengthSent = nbiot.socketSend(socketID, params.getTargetIP(), params.getTargetPort(), buffer, size);

    // wait for data
    if (nbiot.waitForUDPResponse()) {
        debugPrintLn("Received UDP response...");
        while (nbiot.hasPendingUDPBytes()) {
            uint8_t data[128];
            int size = nbiot.socketReceiveBytes(data, sizeof(data));
            if (size) {
                _callback(data, size);
            }
            else {
                debugPrintLn("Receive failed!");
            }
        }
    }
    else {
        debugPrintLn("Timed-out waiting for UDP Response!");
    }

    nbiot.closeSocket(socketID);
    debugPrintLn();

    setActive(false);

    return lengthSent;
}

void NbiotNetwork::loopHandler()
{
    // TODO: add support for retransmission ?
}

void NbiotNetwork::sleep()
{
    // goes to sleep automatically

}

bool NbiotNetwork::getIMEI(char* buffer, size_t size)
{
    return nbiot.getIMEI(buffer, size);
}

bool NbiotNetwork::getModuleVersion(char* buffer, size_t size)
{
    return nbiot.getFirmwareVersion(buffer, size);
}
