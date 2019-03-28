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
#define SODAQ_N2X_DEFAULT_CID           1
#define SODAQ_N2X_DEFAULT_UDP_TIMOUT_MS 15000
#define SODAQ_N2X_MAX_UDP_BUFFER        256

#include "Arduino.h"

enum GSMResponseTypes {
    GSMResponseNotFound = 0,
    GSMResponseOK = 1,
    GSMResponseError = 2,
    GSMResponsePrompt = 3,
    GSMResponseTimeout = 4,
    GSMResponseEmpty = 5
};

enum Protocols {
    TCP = 0,
    UDP
};

enum SentMessageStatus {
    Pending,
    Error
};

struct ReceivedMessageStatus {
    uint16_t pending;
    uint16_t receivedSinceBoot;
    uint16_t droppedSinceBoot;
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
    /******************************************************************************
    * Main
    *****************************************************************************/

    Sodaq_N2X();

    // Initializes the modem instance. Sets the modem stream and the on-off power pins.
    void init(Sodaq_OnOffBee* onoff, Stream& stream, uint8_t cid = SODAQ_N2X_DEFAULT_CID);

    // Turns the modem on/off and returns true if successful.
    bool on();
    bool off();

    // Turns on and initializes the modem, then connects to the network and activates the data connection.
    bool connect(const char* apn, const char* cdp, const char* forceOperator = 0, uint8_t band = 8);

    // Disconnects the modem from the network.
    bool disconnect();

    // Returns the default baud rate of the modem.
    // To be used when initializing the modem stream for the first time.
    uint32_t getDefaultBaudrate() { return 9600; };

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream &stream) { _diagStream = &stream; }
    void setDiag(Stream *stream) { _diagStream = stream; }

    // Sets the size of the input buffer.
    // Needs to be called before init().
    void setInputBufferSize(size_t value) { _inputBufferSize = value; };


    /******************************************************************************
    * Public
    *****************************************************************************/

    bool attachGprs(uint32_t timeout = 10L * 60L * 1000);
    bool getCCID(char* buffer, size_t size);
    bool getEpoch(uint32_t* epoch);
    bool getFirmwareVersion(char* buffer, size_t size);
    bool getIMEI(char* buffer, size_t size);
    bool execCommand(const char* command, uint32_t timeout = DEFAULT_READ_MS, char* buffer = NULL, size_t size = 0);

    // Returns true if the modem replies to "AT" commands without timing out.
    bool isAlive();

    // Returns true if the modem is attached to the network and has an activated data connection.
    bool isAttached();

    // Returns true if the modem is connected to the network and IP address is not 0.0.0.0.
    bool isConnected();

    bool overrideNconfigParam(const char* param, bool value);
    bool ping(const char* ip);
    void purgeAllResponsesRead();
    bool setApn(const char* apn);
    bool setBand(uint8_t band);
    bool setCdp(const char* cdp);
    bool setIndicationsActive(bool on);
    bool setOperator(const char* opr);
    bool setRadioActive(bool on);
    bool setVerboseErrors(bool on);


    /******************************************************************************
    * RSSI and CSQ
    *****************************************************************************/

    int8_t  convertCSQ2RSSI(uint8_t csq) const;
    uint8_t convertRSSI2CSQ(int8_t rssi) const;

    uint8_t getCSQtime()  const { return _CSQtime; }
    int8_t  getLastRSSI() const { return _lastRSSI; }
    int8_t  getMinRSSI()  const { return _minRSSI; }

    // Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
    // Returns true if successful.
    bool    getRSSIAndBER(int8_t* rssi, uint8_t* ber);

    void    setMinCSQ(int csq) { _minRSSI = convertCSQ2RSSI(csq); }
    void    setMinRSSI(int rssi) { _minRSSI = rssi; }


    /******************************************************************************
    * Sockets
    *****************************************************************************/

    int    socketCreate(uint16_t localPort = 0, Protocols protocol = UDP);

    size_t socketSend(uint8_t socketID, const char* remoteHost, const uint16_t remotePort, const uint8_t* buffer, size_t size);
    bool   socketWaitForReceive(uint8_t socketID, uint32_t timeout = SODAQ_N2X_DEFAULT_UDP_TIMOUT_MS);
    size_t socketReceive(uint8_t socketID, uint8_t* buffer, size_t length);

    bool   socketClose(uint8_t socketID, bool async = false);

    size_t socketGetPendingBytes(uint8_t socketID);
    bool   socketHasPendingBytes(uint8_t socketID);


    /******************************************************************************
    * Messsages
    *****************************************************************************/

    bool   getReceivedMessagesCount(ReceivedMessageStatus* status);
    int    getSentMessagesCount(SentMessageStatus filter);
    size_t receiveMessage(char* buffer, size_t size);
    bool   sendMessage(const uint8_t* buffer, size_t size);
    bool   sendMessage(const char* str);
    bool   sendMessage(String str);

private:
    /******************************************************************************
    * Private
    *****************************************************************************/

    uint8_t _cid;
    size_t  _socketPendingBytes[SOCKET_COUNT];

    bool   checkAndApplyNconfig();
    bool   checkURC(char* buffer);

    GSMResponseTypes readResponse(char* outBuffer = NULL, size_t outMaxSize = 0, const char* prefix = NULL,
                                  uint32_t timeout = DEFAULT_READ_MS);

    void   reboot();
    bool   setNconfigParam(const char* param, const char* value);
    bool   waitForSignalQuality(uint32_t timeout = 5L * 60L * 1000);


    /******************************************************************************
    * Utils
    *****************************************************************************/

    static uint32_t convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec);
    static bool startsWith(const char* pre, const char* str);


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
