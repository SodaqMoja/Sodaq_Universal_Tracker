/*
Copyright (c) 2015-18, SODAQ
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

#ifndef _SODAQ_AT_DEVICE_h
#define _SODAQ_AT_DEVICE_h

#include <Arduino.h>
#include <stdint.h>
#include <Stream.h>
#include "Sodaq_OnOffBee.h"

// Response type (returned by readResponse() and parser methods).
enum ResponseTypes {
    ResponseNotFound = 0,
    ResponseOK = 1,
    ResponseError = 2,
    ResponsePrompt = 3,
    ResponseTimeout = 4,
    ResponseEmpty = 5,
    ResponsePendingExtra = 6,
};

// IP type
typedef uint32_t IP_t;

// callback for changing the baudrate of the modem stream.
typedef void (*BaudRateChangeCallbackPtr)(uint32_t newBaudrate);

#define SODAQ_AT_DEVICE_DEFAULT_READ_MS 5000 // Used in readResponse()

class Sodaq_AT_Device
{
  public:
    // Constructor
    Sodaq_AT_Device();
    virtual ~Sodaq_AT_Device() {}

    // Sets the onoff instance
    void setOnOff(Sodaq_OnOffBee& onoff) { _onoff = &onoff; }

    // Turns the modem on and returns true if successful.
    bool on();

    // Turns the modem off and returns true if successful.
    bool off();

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream& stream) { _diagStream = &stream; }
    void setDiag(Stream* stream) { _diagStream = stream; }

    // Sets the size of the input buffer.
    // Needs to be called before init().
    void setInputBufferSize(size_t value) { this->_inputBufferSize = value; };

    // Returns the default baud rate of the modem.
    // To be used when initializing the modem stream for the first time.
    virtual uint32_t getDefaultBaudrate() = 0;

    // Enables the change of the baud rate to a higher speed when the modem is ready to do so.
    // Needs a callback in the main application to re-initialize the stream.
    void enableBaudrateChange(BaudRateChangeCallbackPtr callback) { _baudRateChangeCallbackPtr = callback; };

  protected:
    // the (optional) tx enable pin.
    int8_t _txEnablePin;

    // The stream that communicates with the device.
    Stream* _modemStream;

    // The (optional) stream to show debug information.
    Stream* _diagStream;
    bool _disableDiag;

    // The size of the input buffer. Equals SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    size_t _inputBufferSize;

    // Flag to make sure the buffers are not allocated more than once.
    bool _isBufferInitialized;

    // The buffer used when reading from the modem. The space is allocated during init() via initBuffer().
    char* _inputBuffer;

    // The on-off pin power controller object.
    Sodaq_OnOffBee* _onoff;

    // The callback for requesting baudrate change of the modem stream.
    BaudRateChangeCallbackPtr _baudRateChangeCallbackPtr;

    // This flag keeps track if the next write is the continuation of the current command
    // A Carriage Return will reset this flag.
    bool _appendCommand;

    // Keep track when connect started. Use this to record various status changes.
    uint32_t _startOn;

    // Initializes the input buffer and makes sure it is only initialized once.
    // Safe to call multiple times.
    void initBuffer();

    // Returns true if the modem is ON (and replies to "AT" commands without timing out)
    virtual bool isAlive() = 0;

    // Returns true if the modem is on.
    bool isOn() const;

    // Sets the modem stream.
    void setModemStream(Stream& stream);

    // Sets the optional tx enable pin.
    void setTxEnablePin(int8_t txEnablePin);

    // Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
    int timedRead(uint32_t timeout = 1000) const;

    // Fills the given "buffer" with characters read from the modem stream up to "length"
    // maximum characters and until the "terminator" character is found or a character read
    // times out (whichever happens first).
    // The buffer does not contain the "terminator" character or a null terminator explicitly.
    // Returns the number of characters written to the buffer, not including null terminator.
    size_t readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout = 1000);

    // Fills the given "buffer" with up to "length" characters read from the modem stream.
    // It stops when a character read times out or "length" characters have been read.
    // Returns the number of characters written to the buffer.
    size_t readBytes(uint8_t* buffer, size_t length, uint32_t timeout = 1000);

    // Reads a line from the modem stream into the "buffer". The line terminator is not
    // written into the buffer. The buffer is terminated with null.
    // Returns the number of bytes read, not including the null terminator.
    size_t readLn(char* buffer, size_t size, uint32_t timeout = 1000);

    // Reads a line from the modem stream into the input buffer.
    // Returns the number of bytes read.
    size_t readLn() { return readLn(_inputBuffer, _inputBufferSize); };

    // Write a byte
    size_t writeByte(uint8_t value);

    // Enables or disables the tx power pin, if that is available (!=-1)
    void setTxPowerIfAvailable(bool on);

    // Write the command prolog (just for debugging and/or enabling tx power)
    void writeProlog();

    size_t print(const __FlashStringHelper*);
    size_t print(const String&);
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);
    size_t print(const Printable&);

    size_t println(const __FlashStringHelper*);
    size_t println(const String& s);
    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(double, int = 2);
    size_t println(const Printable&);
    size_t println(void);

    virtual ResponseTypes readResponse(char* buffer, size_t size, size_t* outSize, uint32_t timeout = SODAQ_AT_DEVICE_DEFAULT_READ_MS) = 0;
};

#endif
