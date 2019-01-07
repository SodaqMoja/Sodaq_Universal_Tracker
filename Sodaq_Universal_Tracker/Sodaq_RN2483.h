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

#ifndef SODAQ_RN2483_H_
#define SODAQ_RN2483_H_

#include <Arduino.h>

#include <stdint.h>
#include <Stream.h>

/**

    Notes:

    - uint16_t is preferred over size_t because long is never needed by the
    size of the packets or the buffers of this application.
    (Kees Bakker does not agree with this. size_t is not the same as long.
    On AVR a size_t is uint16_t. On SAMD we don't care too much about the
    data size, a long is fine.)
    - Currently, only one received packet is supported. Every time a packet is
    received, the previous one is overwritten.
    - Also multiple responses from the server (with Frame Pending Bit set) are
    not supported.
    - The port of the received packet is not returned.

*/

//#define USE_DYNAMIC_BUFFER

#define DEFAULT_INPUT_BUFFER_SIZE 64
#define DEFAULT_RECEIVED_PAYLOAD_BUFFER_SIZE 32
#define MAX_VERSION_LENGTH 12
#define DEFAULT_CHAR_TIMEOUT 200
#define DEFAULT_TIMEOUT 400
#define RECEIVE_TIMEOUT 60000
#define DEFAULT_FSB 2
#define DEFAULT_PWR_IDX_868 1
#define DEFAULT_PWR_IDX_915 5
#define DEFAULT_SF_868 12
#define DEFAULT_SF_915 9

#if defined(ARDUINO_ARCH_AVR)
typedef HardwareSerial SerialType;
#define ENABLE_SLEEP
#elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
typedef Uart SerialType;
#define ENABLE_SLEEP
#else
typedef Stream SerialType;
#endif

// Available error codes.
enum MacTransmitErrorCodes {
    NoError = 0,
    NoResponse = 1,
    Timeout = 2,
    PayloadSizeError = 3,
    InternalError = 4,
    Busy = 5,
    NetworkFatalError = 6,
    NotConnected = 7,
    NoAcknowledgment = 8,
};

// Provides a simple, abstracted interface to Microchip's RN2483 LoRaWAN module.
//
// It is strongly suggested to use the static instance that is included with the library (LoRaBee)
// and not to create a new instance.
class Sodaq_RN2483
{
  public:
    typedef void(*ReceiveCallback)(const uint8_t* buffer, uint16_t size);

    // Creates a new Sodaq_RN2483 instance.
    Sodaq_RN2483();

    // Returns the correct baudrate for the serial port that connects to the device.
    uint32_t getDefaultBaudRate() { return 57600; };

    uint8_t getVersion(char* version, uint8_t size);

    // Takes care of the initialization tasks common to both initOTA() and initABP().
    // If hardware reset is available, the module is re-set, otherwise it is woken up if possible.
    // Returns true if the module replies to a device reset command.
    bool init(SerialType& stream, int8_t resetPin = -1);

    // Initializes the device and connects to the network using Over-The-Air Activation.
    // Returns true on successful connection.
    bool initOTA(SerialType& stream, const uint8_t devEUI[8], const uint8_t appEUI[8], const uint8_t appKey[16], bool adr = true, int8_t resetPin = -1);
    bool initOTA(const uint8_t devEUI[8], const uint8_t appEUI[8], const uint8_t appKey[16], bool adr = true);

    // Initializes the device and connects to the network using Activation By Personalization.
    // Returns true on successful connection.
    bool initABP(SerialType& stream, const uint8_t devAddr[4], const uint8_t appSKey[16], const uint8_t nwkSKey[16], bool adr = true, int8_t resetPin = -1);
    bool initABP(const uint8_t devAddr[4], const uint8_t appSKey[16], const uint8_t nwkSKey[16], bool adr = true);

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream& stream) { _diagStream = &stream; };

    // Performs a hardware reset (using the reset pin -if available).
    void hardwareReset();

    // Sends the given payload without acknowledgement.
    // Returns 0 (NoError) when transmission is successful or one of the MacTransmitErrorCodes otherwise.
    uint8_t send(uint8_t port, const uint8_t* payload, uint8_t size);

    // Sends the given payload with acknowledgement.
    // Returns 0 (NoError) when transmission is successful or one of the MacTransmitErrorCodes otherwise.
    uint8_t sendReqAck(uint8_t port, const uint8_t* payload, uint8_t size, uint8_t maxRetries);

    // Copies the latest received packet (optionally starting from the "payloadStartPosition"
    // position of the payload) into the given "buffer", up to "size" number of bytes.
    // Returns the number of bytes written or 0 if no packet is received since last transmission.
    uint16_t receive(uint8_t* buffer, uint16_t size, uint16_t payloadStartPosition = 0);

    // Gets the preprogrammed EUI node address from the module.
    // Returns the number of bytes written or 0 in case of error.
    uint8_t getHWEUI(uint8_t* buffer, uint8_t size);

    // Enables all the channels that belong to the given Frequency Sub-Band (FSB)
    // and disables the rest.
    // fsb is [1, 8] or 0 to enable all channels.
    // Returns true if all channels were set successfully.
    bool setFsbChannels(uint8_t fsb);

    // Sets the spreading factor.
    // In reality it sets the datarate of the module according to the
    // LoraWAN specs mapping for 868MHz and 915MHz,
    // using the given spreading factor parameter.
    bool setSpreadingFactor(uint8_t spreadingFactor);

    // Sets the power index (868MHz: 1 to 5 / 915MHz: 5, 7, 8, 9 or 10)
    // Returns true if successful.
    bool setPowerIndex(uint8_t powerIndex);

    // Sends the command together with the given paramValue (optional)
    // to the device and awaits for the response.
    // Returns true on success.
    // NOTE: command should include a trailing space if paramValue is set
    bool sendCommand(const char* command, const uint8_t* paramValue, uint16_t size);
    bool sendCommand(const char* command, uint8_t paramValue);
    bool sendCommand(const char* command, const char* paramValue = NULL);

    // Sends the given mac command together with the given paramValue
    // to the device and awaits for the response.
    // Returns true on success.
    // NOTE: paramName should include a trailing space
    bool setMacParam(const char* paramName, const uint8_t* paramValue, uint16_t size);
    bool setMacParam(const char* paramName, uint8_t paramValue);
    bool setMacParam(const char* paramName, const char* paramValue);

    // Sets the (optional) callback to call when a reply is received
    void setReceiveCallback(ReceiveCallback callback) { _receiveCallback = callback; };

#ifdef ENABLE_SLEEP
    // Wakes up the module from sleep (if supported).
    void wakeUp();

    // Puts the module to sleep (if supported).
    void sleep();
#endif

#ifdef USE_DYNAMIC_BUFFER
    // Sets the size of the input buffer.
    // Needs to be called before initOTA()/initABP().
    void setInputBufferSize(uint16_t value) { this->_inputBufferSize = value; };

    // Sets the size of the "Received Payload" buffer.
    // Needs to be called before initOTA()/initABP().
    void setReceivedPayloadBufferSize(uint16_t value) { this->_receivedPayloadBufferSize = value; };
#endif

    // Provides a quick test of several methods as a pseudo-unit test.
    void runTestSequence(SerialType& loraStream, Stream& debugStream);

  private:
    // The stream that communicates with the device.
    SerialType* _loraStream;

    // The (optional) stream to show debug information.
    Stream* _diagStream;

    // The size of the input buffer. Equals DEFAULT_INPUT_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    uint16_t _inputBufferSize;

    // The size of the received payload buffer. Equals DEFAULT_RECEIVED_PAYLOAD_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    uint16_t _receivedPayloadBufferSize;

    // The size of the last received payload.
    uint16_t _receivedPayloadSize;

    // The buffer where the version is saved (during resetDevice()).
    char _version[MAX_VERSION_LENGTH];

    // Flag used to make sure the received payload buffer is
    // current with the latest transmission.
    bool _packetReceived;

    // Used to distinguise between RN2483 and RN2903.
    // Currently only being set during reset().
    bool _isRN2903;

#ifdef USE_DYNAMIC_BUFFER
    // Flag to make sure the buffers are not allocated more than once.
    bool _isBufferInitialized;

    char* _inputBuffer;
    char* _receivedPayloadBuffer;
#else
    char _inputBuffer[DEFAULT_INPUT_BUFFER_SIZE];
    char _receivedPayloadBuffer[DEFAULT_RECEIVED_PAYLOAD_BUFFER_SIZE];
#endif

    // Used for resetting the module on init.
    int8_t _resetPin;

    // This flag keeps track if the next write is the continuation of the current command
    // A Carriage Return will reset this flag.
    bool _appendCommand;

    // (optional) callback to call when a reply is received
    ReceiveCallback _receiveCallback;

    // Enables hardware-resetting the module.
    void enableHardwareReset(uint8_t resetPin) { this->_resetPin = resetPin; };

    // Returns true if the hardware reset pin is set.
    bool isHardwareResetEnabled() { return _resetPin >= 0; };

    // Reads a line from the device stream into the "buffer" starting at the "start" position of the buffer.
    // Returns the number of bytes read.
    uint16_t readLn(char* buffer, uint16_t size, uint16_t start = 0);

    // Reads a line from the device stream into the input buffer.
    // Returns the number of bytes read.
    uint16_t readLn() { return readLn(this->_inputBuffer, this->_inputBufferSize); };

    // Write a byte
    size_t writeByte(uint8_t value);

    // Write the command prolog (just for debugging
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

    // Waits for the given string. Returns true if the string is received before a timeout.
    // Returns false if a timeout occurs or if another string is received.
    bool expectString(const char* str, uint16_t timeout = DEFAULT_TIMEOUT);
    bool expectOK();

    // Sends a reset command to the module and waits for the success response (or timeout).
    // Returns true on success.
    bool resetDevice();

    // Sends a join network command to the device and waits for the response (or timeout).
    // Returns true on success.
    bool joinNetwork(const char* type);

    // Returns the enum that is mapped to the given "error" message.
    uint8_t lookupMacTransmitError(const char* error);

    // Sends a a payload and blocks until there is a response back, or the receive windows have closed,
    // or the hard timeout has passed.
    uint8_t macTransmit(const char* type, uint8_t port, const uint8_t* payload, uint8_t size);

    // Parses the input buffer and copies the received payload into the "received payload" buffer
    // when a "mac rx" message has been received. It is called internally by macTransmit().
    // Returns 0 (NoError) or otherwise one of the MacTransmitErrorCodes.
    uint8_t onMacRX();

    // Used during deviceReset() to copy the version information to the version buffer.
    void fillVersionFromReceivedBuffer();
};

extern Sodaq_RN2483 LoRaBee;

#endif // SODAQ_RN2483_H
