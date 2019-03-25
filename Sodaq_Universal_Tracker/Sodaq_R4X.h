/*
    Copyright (c) 2019 Sodaq.  All rights reserved.

    This file is part of Sodaq_R4X.

    Sodaq_R4X is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of
    the License, or(at your option) any later version.

    Sodaq_R4X is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with Sodaq_nbIOT.  If not, see
    <http://www.gnu.org/licenses/>.
*/

#ifndef _Sodaq_R4X_h
#define _Sodaq_R4X_h

#define DEFAULT_READ_MS                 5000
#define SODAQ_MAX_UDP_SEND_MESSAGE_SIZE 512
#define SODAQ_R4X_DEFAULT_CID           1
#define SODAQ_R4X_DEFAULT_UDP_TIMOUT_MS 15000
#define SODAQ_R4X_MAX_UDP_BUFFER        256

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

enum HttpRequestTypes {
    POST = 0,
    GET,
    HEAD,
    DELETE,
    PUT,
    HttpRequestTypesMAX
};

enum SimStatuses {
    SimStatusUnknown = 0,
    SimMissing,
    SimNeedsPin,
    SimReady,
};

enum TriBoolStates
{
    TriBoolFalse,
    TriBoolTrue,
    TriBoolUndefined
};

typedef TriBoolStates tribool_t;

#define UNUSED(x) (void)(x)

#define SOCKET_COUNT 7

class Sodaq_OnOffBee
{
public:
    virtual ~Sodaq_OnOffBee() {}
    virtual void on()   = 0;
    virtual void off()  = 0;
    virtual bool isOn() = 0;
};

class Sodaq_SARA_R4XX_OnOff : public Sodaq_OnOffBee
{
public:
    Sodaq_SARA_R4XX_OnOff();
    void on();
    void off();
    bool isOn();
private:
    bool _onoff_status;
};

class Sodaq_R4X
{
public:
    /******************************************************************************
    * Main
    *****************************************************************************/

    Sodaq_R4X();

    // Initializes the modem instance. Sets the modem stream and the on-off power pins.
    void init(Sodaq_OnOffBee* onoff, Stream& stream, uint8_t cid = SODAQ_R4X_DEFAULT_CID);

    // Turns the modem on/off and returns true if successful.
    bool on();
    bool off();

    // Turns on and initializes the modem, then connects to the network and activates the data connection.
    bool connect(const char* apn, const char* urat = 0, const char* bandMask = 0);

    // Disconnects the modem from the network.
    bool disconnect();

    // Returns the default baud rate of the modem.
    // To be used when initializing the modem stream for the first time.
    uint32_t getDefaultBaudrate() { return 115200; };

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
    SimStatuses getSimStatus();
    bool execCommand(const char* command, uint32_t timeout = DEFAULT_READ_MS, char* buffer = NULL, size_t size = 0);

    // Returns true if the modem replies to "AT" commands without timing out.
    bool isAlive();

    // Returns true if the modem is attached to the network and has an activated data connection.
    bool isAttached();

    // Returns true if the modem is connected to the network and IP address is not 0.0.0.0.
    bool isConnected();

    // Returns true if defined IP4 address is not 0.0.0.0.
    bool isDefinedIP4();

    void purgeAllResponsesRead();
    bool setApn(const char* apn);
    bool setIndicationsActive(bool on);
    void setPin(const char* pin);
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

    int    createSocket(uint16_t localPort = 0);
    bool   closeSocket(uint8_t socketID);
    size_t getPendingUDPBytes(uint8_t socketID);
    bool   hasPendingUDPBytes(uint8_t socketID);
    size_t socketReceiveBytes(uint8_t socketID, uint8_t* buffer, size_t length, SaraN2UDPPacketMetadata* p = NULL);
    size_t socketReceiveHex(uint8_t socketID, char* buffer, size_t length, SaraN2UDPPacketMetadata* p = NULL);
    size_t socketSend(uint8_t socketID, const char* remoteIP, const uint16_t remotePort, const char* str);
    size_t socketSend(uint8_t socketID, const char* remoteIP, const uint16_t remotePort, const uint8_t* buffer, size_t size);
    bool   waitForUDPResponse(uint8_t socketID, uint32_t timeoutMS = SODAQ_R4X_DEFAULT_UDP_TIMOUT_MS);


    /******************************************************************************
    * MQTT
    *****************************************************************************/

    bool mqttLogin(uint32_t timeout = 3 * 60 * 1000);
    bool mqttLogout();
    void mqttLoop();
    bool mqttPing(const char* server);
    bool mqttPublish(const char* topic, const uint8_t* msg, size_t size, uint8_t qos = 0, uint8_t retain = 0, bool useHEX = false);
    uint16_t mqttReadMessages(char* buffer, size_t size, uint32_t timeout = 60 * 1000);
    bool mqttSubscribe(const char* filter, uint8_t qos = 0, uint32_t timeout = 30 * 1000);
    bool mqttUnsubscribe(const char* filter);

    int8_t  mqttGetLoginResult();
    int16_t mqttGetPendingMessages();

    bool mqttSetAuth(const char* name, const char* pw);
    bool mqttSetCleanSettion(bool enabled);
    bool mqttSetClientId(const char* id);
    bool mqttSetInactivityTimeout(uint16_t timeout);
    bool mqttSetLocalPort(uint16_t port);
    bool mqttSetSecureOption(bool enabled, int8_t profile = -1);
    bool mqttSetServer(const char* server, uint16_t port);
    bool mqttSetServerIP(const char* ip, uint16_t port);


    /******************************************************************************
    * HTTP
    *****************************************************************************/

    // Creates an HTTP GET request and optionally returns the received data.
    // Note. Endpoint should include the initial "/".
    // The UBlox device stores the received data in http_last_response_<profile_id>
    uint32_t httpGet(const char* server, uint16_t port, const char* endpoint,
                     char* buffer, size_t bufferSize, uint32_t timeout = 60000, bool useURC = true);

    // Determine HTTP header size
    uint32_t httpGetHeaderSize(const char* filename);

    // Return a partial result of the previous HTTP Request (GET or POST)
    // Offset 0 is the byte directly after the HTTP Response header
    size_t httpGetPartial(uint8_t* buffer, size_t size, uint32_t offset);

    // Creates an HTTP request using the (optional) given buffer and
    // (optionally) returns the received data.
    // endpoint should include the initial "/".
    size_t httpRequest(const char* server, uint16_t port, const char* endpoint,
                       HttpRequestTypes requestType = HttpRequestTypes::GET,
                       char* responseBuffer = NULL, size_t responseSize = 0,
                       const char* sendBuffer = NULL, size_t sendSize = 0, uint32_t timeout = 60000, bool useURC = true);

    /******************************************************************************
    * Files
    *****************************************************************************/

    bool   deleteFile(const char* filename);
    bool   getFileSize(const char* filename, uint32_t& size);
    size_t readFile(const char* filename, uint8_t* buffer, size_t size);
    size_t readFilePartial(const char* filename, uint8_t* buffer, size_t size, uint32_t offset);

    // If the file already exists, the data will be appended to the file already stored in the file system.
    bool   writeFile(const char* filename, const uint8_t* buffer, size_t size);


private:
    /******************************************************************************
    * Private
    *****************************************************************************/

    uint8_t   _cid;
    uint32_t  _httpGetHeaderSize;
    tribool_t _httpRequestSuccessBit[HttpRequestTypesMAX];
    int8_t    _mqttLoginResult;
    int16_t   _mqttPendingMessages;
    int8_t    _mqttSubscribeReason;
    size_t    _pendingUDPBytes[SOCKET_COUNT];
    char*     _pin;

    int8_t checkApn(const char* requiredAPN); // -1: error, 0: ip not valid => need attach, 1: valid ip
    bool   checkBandMask(const char* requiredURAT, const char* requiredBankMask);
    bool   checkCFUN();
    bool   checkCOPS();
    bool   checkUrat(const char* requiredURAT);
    bool   checkURC(char* buffer);
    bool   doSIMcheck();
    bool   isValidIPv4(const char* str);

    GSMResponseTypes readResponse(char* outBuffer = NULL, size_t outMaxSize = 0, const char* prefix = NULL,
                                  uint32_t timeout = DEFAULT_READ_MS);

    void   reboot();
    bool   setSimPin(const char* simPin);
    size_t socketReceive(uint8_t socketID, SaraN2UDPPacketMetadata* packet, char* buffer, size_t size);
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

    char* _apn;
    char* _apnUser;
    char* _apnPass;

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
