/*
    Copyright (c) 2019 Sodaq.  All rights reserved.

    This file is part of Sodaq_N2X.

    Sodaq_N2X is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of
    the License, or(at your option) any later version.

    Sodaq_N2X is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with Sodaq_N2X.  If not, see
    <http://www.gnu.org/licenses/>.
*/

#ifndef _Sodaq_N2X_h
#define _Sodaq_N2X_h

#define DEFAULT_READ_MS                 5000
#define SODAQ_MAX_UDP_SEND_MESSAGE_SIZE 512
#define SODAQ_N2X_DEFAULT_UDP_TIMOUT_MS 15000
#define SODAQ_N2X_MAX_UDP_BUFFER        256
#define SODAQ_N2X_DEFAULT_CID           0

#include "Arduino.h"

struct SaraN2UDPPacketMetadata {
    uint8_t socketID;
    char ip[16]; // max IP size 4*3 digits + 3 dots + zero term = 16
    int port;
    int length;
    int remainingLength;
};

enum GSMResponseTypes {
    GSMResponseNotFound = 0,
    GSMResponseOK = 1,
    GSMResponseError = 2,
    GSMResponsePrompt = 3,
    GSMResponseTimeout = 4,
    GSMResponseEmpty = 5
};

#define UNUSED(x) (void)(x)

typedef uint32_t IP_t;

#define SOCKET_COUNT 7

class Sodaq_OnOffBee
{
public:
    virtual ~Sodaq_OnOffBee() {}
    virtual void on()   = 0;
    virtual void off()  = 0;
    virtual bool isOn() = 0;
};

class Sodaq_SARA_N211_OnOff : public Sodaq_OnOffBee
{
public:
    Sodaq_SARA_N211_OnOff();
    void on();
    void off();
    bool isOn();
private:
    bool _onoff_status;
};

class Sodaq_N2X
{
public:
    Sodaq_N2X();

    enum SentMessageStatus {
        Pending,
        Error
    };

    struct ReceivedMessageStatus {
        uint16_t pending;
        uint16_t receivedSinceBoot;
        uint16_t droppedSinceBoot;
    };

    // Turns the modem on and returns true if successful.
    bool on();

    // Turns the modem off and returns true if successful.
    bool off();

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream &stream) { _diagStream = &stream; }
    void setDiag(Stream *stream) { _diagStream = stream; }

    // Sets the size of the input buffer.
    // Needs to be called before init().
    void setInputBufferSize(size_t value) { _inputBufferSize = value; };

    bool setRadioActive(bool on);
    bool setIndicationsActive(bool on);
    bool setApn(const char* apn);
    bool setCdp(const char* cdp);
    bool getEpoch(uint32_t* epoch);
    bool setBand(uint8_t band);
    bool setVerboseErrors(bool on);
    bool setOperator(const char* opr);

    // Returns true if the modem replies to "AT" commands without timing out.
    bool isAlive();

    // Returns the default baud rate of the modem.
    // To be used when initializing the modem stream for the first time.
    uint32_t getDefaultBaudrate() { return 9600; };

    // Initializes the modem instance. Sets the modem stream and the on-off power pins.
    void init(Sodaq_OnOffBee* onoff, Stream& stream, uint8_t cid = SODAQ_N2X_DEFAULT_CID);

    bool overrideNconfigParam(const char* param, bool value);

    // Turns on and initializes the modem, then connects to the network and activates the data connection.
    bool connect(const char* apn, const char* cdp, const char* forceOperator = 0, uint8_t band = 8);

    // Disconnects the modem from the network.
    bool disconnect();

    // Returns true if the modem is attached to the network and has an activated data connection.
    bool isAttached();

    // Returns true if the modem is connected to the network and IP address is not 0.0.0.0.
    bool isConnected();

    // Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
    // Returns true if successful.
    bool getRSSIAndBER(int8_t* rssi, uint8_t* ber);
    int8_t convertCSQ2RSSI(uint8_t csq) const;
    uint8_t convertRSSI2CSQ(int8_t rssi) const;

    void setMinRSSI(int rssi) { _minRSSI = rssi; }
    void setMinCSQ(int csq) { _minRSSI = convertCSQ2RSSI(csq); }
    int8_t getMinRSSI() const { return _minRSSI; }
    uint8_t getCSQtime() const { return _CSQtime; }
    int8_t getLastRSSI() const { return _lastRSSI; }

    int createSocket(uint16_t localPort = 0);
    bool closeSocket(uint8_t socketID);
    size_t socketSend(uint8_t socketID, const char* remoteIP, const uint16_t remotePort, const uint8_t* buffer, size_t size);
    size_t socketSend(uint8_t socketID, const char* remoteIP, const uint16_t remotePort, const char* str);
    size_t socketReceiveHex(uint8_t socketID, char* buffer, size_t length, SaraN2UDPPacketMetadata* p = NULL);
    size_t socketReceiveBytes(uint8_t socketID, uint8_t* buffer, size_t length, SaraN2UDPPacketMetadata* p = NULL);
    size_t getPendingUDPBytes(uint8_t socketID);
    bool hasPendingUDPBytes(uint8_t socketID);
    bool waitForUDPResponse(uint8_t socketID, uint32_t timeoutMS = SODAQ_N2X_DEFAULT_UDP_TIMOUT_MS);

    bool ping(const char* ip);

    bool sendMessage(const uint8_t* buffer, size_t size);
    bool sendMessage(const char* str);
    bool sendMessage(String str);
    size_t receiveMessage(char* buffer, size_t size);

    int getSentMessagesCount(SentMessageStatus filter);
    bool getReceivedMessagesCount(ReceivedMessageStatus* status);

    bool getIMEI(char* buffer, size_t size);
    bool getCCID(char* buffer, size_t size);
    bool getFirmwareVersion(char* buffer, size_t size);

    void purgeAllResponsesRead();
    bool attachGprs(uint32_t timeout = 10L * 60L * 1000);
    bool execCommand(const char* command, uint32_t timeout = DEFAULT_READ_MS, char* buffer = NULL, size_t size = 0);

private:
    uint8_t _cid;

    // flag indicating UDP response via URC
    size_t _pendingUDPBytes[SOCKET_COUNT];

    static bool startsWith(const char* pre, const char* str);

    bool waitForSignalQuality(uint32_t timeout = 5L * 60L * 1000);
    bool setNconfigParam(const char* param, const char* value);
    bool checkAndApplyNconfig();
    bool checkURC(char* buffer);
    void reboot();

    GSMResponseTypes readResponse(char* outBuffer = NULL, size_t outMaxSize = 0, const char* prefix = NULL, uint32_t timeout = DEFAULT_READ_MS);

    size_t socketReceive(uint8_t socketID, SaraN2UDPPacketMetadata* packet, char* buffer, size_t size);
    static uint32_t convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec);


    /******************************************************************************
     * Generic
     *****************************************************************************/

    // The on-off pin power controller object.
    Sodaq_OnOffBee* _onoff;

    // The stream that communicates with the device.
    Stream* _modemStream;

    // The (optional) stream to show debug information.
    Stream* _diagStream;

    // The size of the input buffer. Equals SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    size_t _inputBufferSize;

    // Flag to make sure the buffers are not allocated more than once.
    bool _isBufferInitialized;

    // The buffer used when reading from the modem. The space is allocated during init() via initBuffer().
    char* _inputBuffer;

    char * _apn;
    char * _apnUser;
    char * _apnPass;

    // This flag keeps track if the next write is the continuation of the current command
    // A Carriage Return will reset this flag.
    bool _appendCommand;

    // This is the value of the most recent CSQ
    // Notice that CSQ is somewhat standard. SIM800/SIM900 and Ublox
    // compute to comparable numbers. With minor deviations.
    // For example SIM800
    //   1              -111 dBm
    //   2...30         -110... -54 dBm
    // For example UBlox
    //   1              -111 dBm
    //   2..30          -109 to -53 dBm
    int8_t _lastRSSI;   // 0 not known or not detectable

    // This is the number of second it took when CSQ was record last
    uint8_t _CSQtime;

    // This is the minimum required RSSI to continue making the connection
    // Use convertCSQ2RSSI if you have a CSQ value
    int _minRSSI;

    // Keep track when connect started. Use this to record various status changes.
    uint32_t _startOn;

    // Initializes the input buffer and makes sure it is only initialized once.
    // Safe to call multiple times.
    void initBuffer();

    // Returns true if the modem is on.
    bool isOn() const;

    // Sets the modem stream.
    void setModemStream(Stream& stream);

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

    // Write the command prolog (just for debugging
    void writeProlog();

    size_t print(const __FlashStringHelper *);
    size_t print(const String &);
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);
    size_t print(const Printable&);

    size_t println(const __FlashStringHelper *);
    size_t println(const String &s);
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
};

#endif
