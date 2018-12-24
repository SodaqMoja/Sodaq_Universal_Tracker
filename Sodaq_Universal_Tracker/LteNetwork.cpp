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

#include "LteNetwork.h"

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

// BEGIN LteM DEFINES
Sodaq_LteM lteM;
// END LteMs DEFINES

#ifndef PIN_SARA_ENABLE
#define SARA_TX_ENABLE 0
#define SARA_RESET 0
#define SARA_ENABLE 0
#define MODEM_ON_OFF_PIN 0
#define SARA_R4XX_TOGGLE 0
#endif // !ARDUINO_SODAQ_SARA

/**
Initializes the LTE module according to the given operation (join or skip).
Returns true if the operation was successful.
*/

bool LteNetwork::init(Uart & modemStream, DataReceiveCallback callback, InitConsoleMessages messages, InitJoin join)
{
    _callback = callback;
    debugPrintLn("Initializing LteM...");

    if (messages == INIT_SHOW_CONSOLE_MESSAGES) {
        consolePrintln("Initializing LteM...");
    }

    if (params.getIsDebugOn()) {
        if (_diagStream) {
            lteM.setDiag(_diagStream);
        }
    }

    _baudRate = lteM.getSaraR4Baudrate();
    debugPrintLn("Init LteM...");
    modemStream.begin(lteM.getSaraR4Baudrate());
    lteM.init(modemStream, SARA_ENABLE, -1, SARA_R4XX_TOGGLE);

    pinMode(SARA_TX_ENABLE, OUTPUT); // may be needed for other SARA boards
    digitalWrite(SARA_TX_ENABLE, HIGH);

    if (join == INIT_JOIN) {
        // trigger connection
        setActive(true);
    }
    else {
        lteM.on();
    }

    bool modemAlive = lteM.isAlive();

    return modemAlive;
}

/**
* Turns the nbiot module on or off (and connects/disconnects)
*/

void LteNetwork::setActive(bool on)
{
    sodaq_wdt_reset();
    // TODO Fix reset mechanism
    if (on) {
        if (!lteM.isConnected()) {
            if (!lteM.connect(params._apn, params._forceOperator)) {
                lteM.off();
                sodaq_wdt_safe_delay(450);
                lteM.on();
                sodaq_wdt_safe_delay(450);

                // try just one last time
                lteM.connect(params._apn, params._forceOperator);
            }
        }
    }
    else {
        lteM.off();
    }
}

uint8_t LteNetwork::transmit(uint8_t * buffer, uint8_t size, uint32_t rxTimeout)
{
    setActive(true);

    debugPrintLn();
    debugPrintLn("Sending message through UDP");

    int localPort = 16666;
    int socketID = lteM.createSocket(localPort);

    if (socketID >= 7 || socketID < 0) {
        debugPrintLn("Failed to create socket");
        return false;
    }

    size_t lengthSent = lteM.socketSend(socketID, params.getTargetIP(), params.getTargetPort(), buffer, size);

    // wait for data
    if (lteM.waitForUDPResponse(rxTimeout)) {
        debugPrintLn("Received UDP response...");
        while (lteM.hasPendingUDPBytes()) {
            uint8_t data[128];
            int size = lteM.socketReceiveBytes(data, sizeof(data));
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

    lteM.closeSocket(socketID);
    debugPrintLn();

    setActive(false);

    return lengthSent;
}

void LteNetwork::loopHandler()
{
    // TODO: add support for retransmission ?
}

void LteNetwork::sleep()
{
    // goes to sleep automatically
}

bool LteNetwork::getIMEI(char* buffer, size_t size)
{
    return lteM.getIMEI(buffer, size);
}

bool LteNetwork::getModuleVersion(char* buffer, size_t size)
{
    return lteM.getFirmwareVersion(buffer, size);
}
