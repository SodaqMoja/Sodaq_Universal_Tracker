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

#ifndef _Sodaq_R4X_h
#define _Sodaq_R4X_h

#define DEFAULT_READ_MS                 5000
#define SODAQ_MAX_SEND_MESSAGE_SIZE     512
#define SODAQ_R4X_DEFAULT_CID           1
#define SODAQ_R4X_DEFAULT_READ_TIMOUT   15000
#define SODAQ_R4X_MAX_SOCKET_BUFFER     1024

#define SODAQ_R4X_LTEM_URAT             "7"
#define SODAQ_R4X_NBIOT_URAT            "8"
#define SODAQ_R4X_2G_URAT               "9"
#define SODAQ_R4X_LTEM_NBIOT_URAT       "7,8"
#define SODAQ_R4X_LTEM_2G_URAT          "7,9"
#define SODAQ_R4X_NBIOT_2G_URAT         "8,9"
#define SODAQ_R4X_LTEM_NBIOT_2G_URAT    "7,8,9"

#define DEFAULT_URAT                    SODAQ_R4X_NBIOT_URAT
#define AUTOMATIC_OPERATOR              "0"
#define BAND_MASK_UNCHANGED             0

#include "Arduino.h"

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

enum MNOProfiles {
    SWD_DEFAULT     = 0,
    SIM_ICCID       = 1,
    ATT             = 2,
    VERIZON         = 3,
    TELSTRA         = 4,
    T_MOBILE_US     = 5,
    CHINA_TELECOM   = 6,
    VODAFONE        = 19,
    STANDARD_EUROPE = 100
};

enum Protocols {
    TCP = 0,
    UDP
};

enum SimStatuses {
    SimStatusUnknown = 0,
    SimMissing,
    SimNeedsPin,
    SimReady
};

enum TriBoolStates
{
    TriBoolFalse,
    TriBoolTrue,
    TriBoolUndefined
};

typedef TriBoolStates tribool_t;

typedef void(*PublishHandlerPtr)(const char* topic, const char* msg);

#define UNUSED(x) (void)(x)
#define BAND_TO_MASK(x) (1 << (x - 1))

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
    bool connect(const char* apn, const char* urat = DEFAULT_URAT, 
        const char* bandMask = BAND_MASK_UNCHANGED);

    bool connect(const char* apn, const char* uratSelect, uint8_t mnoProfile, 
        const char* operatorSelect = AUTOMATIC_OPERATOR, const char* bandMaskLTE = BAND_MASK_UNCHANGED, 
        const char* bandMaskNB = BAND_MASK_UNCHANGED);

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
    bool bandMasktoStr(const uint64_t bandMask, char* str, size_t size);
    bool getCCID(char* buffer, size_t size);
    bool getIMSI(char* buffer, size_t size);
    bool getOperatorInfo(uint16_t* mcc, uint16_t* mnc);
    bool getOperatorInfoString(char* buffer, size_t size);
    bool getCellInfo(uint16_t* tac, uint32_t* cid, uint16_t* urat);
    bool getEpoch(uint32_t* epoch);
    bool getFirmwareVersion(char* buffer, size_t size);
    bool getFirmwareRevision(char* buffer, size_t size);
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
    void setNetworkStatusLED(bool on) { _networkStatusLED = on; };
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

    int    socketCreate(uint16_t localPort = 0, Protocols protocol = UDP);

    bool   socketSetR4KeepAlive(uint8_t socketID);
    bool   socketSetR4Option(uint8_t socketID, uint16_t level, uint16_t optName, uint32_t optValue, uint32_t optValue2 = 0);

    // Required for TCP, optional for UDP (for UDP socketConnect() + socketWrite() == socketSend())
    bool   socketConnect(uint8_t socketID, const char* remoteHost, const uint16_t remotePort);
    size_t socketWrite(uint8_t socketID, const uint8_t* buffer, size_t size);

    // TCP only
    bool   socketWaitForRead(uint8_t socketID, uint32_t timeout = SODAQ_R4X_DEFAULT_READ_TIMOUT);
    size_t socketRead(uint8_t socketID, uint8_t* buffer, size_t length);

    // UDP only
    size_t socketSend(uint8_t socketID, const char* remoteHost, const uint16_t remotePort, const uint8_t* buffer, size_t size);
    bool   socketWaitForReceive(uint8_t socketID, uint32_t timeout = SODAQ_R4X_DEFAULT_READ_TIMOUT);
    size_t socketReceive(uint8_t socketID, uint8_t* buffer, size_t length);

    bool   socketClose(uint8_t socketID, bool async = false);
    int    socketCloseAll();
    bool   socketFlush(uint8_t socketID, uint32_t timeout = 10000);
    bool   socketIsClosed(uint8_t socketID);
    bool   socketWaitForClose(uint8_t socketID, uint32_t timeout);

    size_t socketGetPendingBytes(uint8_t socketID);
    bool   socketHasPendingBytes(uint8_t socketID);


    /******************************************************************************
    * MQTT
    *****************************************************************************/

    int8_t  mqttGetLoginResult();
    int16_t mqttGetPendingMessages();

    bool mqttLogin(uint32_t timeout = 3 * 60 * 1000);
    bool mqttLogout();
    void mqttLoop();
    bool mqttPing(const char* server);
    bool mqttPublish(const char* topic, const uint8_t* msg, size_t size, uint8_t qos = 0, uint8_t retain = 0, bool useHEX = false);
    uint16_t mqttReadMessages(char* buffer, size_t size, uint32_t timeout = 60 * 1000);

    bool mqttSetAuth(const char* name, const char* pw);
    bool mqttSetCleanSession(bool enabled);
    bool mqttSetClientId(const char* id);
    bool mqttSetInactivityTimeout(uint16_t timeout);
    bool mqttSetLocalPort(uint16_t port);
    bool mqttSetSecureOption(bool enabled, int8_t profile = -1);
    bool mqttSetServer(const char* server, uint16_t port);
    bool mqttSetServerIP(const char* ip, uint16_t port);

    bool mqttSubscribe(const char* filter, uint8_t qos = 0, uint32_t timeout = 30 * 1000);
    bool mqttUnsubscribe(const char* filter);
    void mqttSetPublishHandler(PublishHandlerPtr handler);

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

    // Creates an HTTP POST request and optionally returns the received data.
    // Note. Endpoint should include the initial "/".
    // The UBlox device stores the received data in http_last_response_<profile_id>
    uint32_t httpPost(const char* server, uint16_t port, const char* endpoint,
                      char* responseBuffer, size_t responseSize,
                      const char* sendBuffer, size_t sendSize, uint32_t timeout = 60000, bool useURC = true);

    // Creates an HTTP POST request and optionally returns the received data.
    // Note. Endpoint should include the initial "/".
    // The UBlox device stores the received data in http_last_response_<profile_id>
    // Request body must first be prepared in a file on the modem
    uint32_t httpPostFromFile(const char* server, uint16_t port, const char* endpoint,
                      char* responseBuffer, size_t responseSize,
                      const char* fileName, uint32_t timeout = 60000, bool useURC = true);

    // Creates an HTTP request using the (optional) given buffer and
    // (optionally) returns the received data.
    // endpoint should include the initial "/".
    size_t httpRequest(const char* server, uint16_t port, const char* endpoint,
                       HttpRequestTypes requestType = HttpRequestTypes::GET,
                       char* responseBuffer = NULL, size_t responseSize = 0,
                       const char* sendBuffer = NULL, size_t sendSize = 0, uint32_t timeout = 60000, bool useURC = true);

     // Creates an HTTP request using the (optional) given buffer and
     // (optionally) returns the received data.
     // endpoint should include the initial "/".
     // Request body must first be prepared in a file on the modem
     // Can only be used for POST and PUT requests
     size_t httpRequestFromFile(const char* server, uint16_t port, const char* endpoint,
                       HttpRequestTypes requestType = HttpRequestTypes::GET,
                       char* responseBuffer = NULL, size_t responseSize = 0,
                       const char* fileName = NULL, uint32_t timeout = 60000, bool useURC = true);

    //  Paremeter index has a range [0-4]
    //  Parameters 'name' and 'value' can have a maximum length of 64 characters
    //  Parameters 'name' and 'value' must not include the ':' character
    bool httpSetCustomHeader(uint8_t index, const char* name, const char* value);
    bool httpClearCustomHeader(uint8_t index);


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
    bool      _networkStatusLED;
    char*     _pin;
    bool      _socketClosedBit[SOCKET_COUNT];
    size_t    _socketPendingBytes[SOCKET_COUNT];

    PublishHandlerPtr _mqttPublishHandler = NULL;

    int8_t checkApn(const char* requiredAPN); // -1: error, 0: ip not valid => need attach, 1: valid ip
    bool   checkBandMasks(const char* bandMaskLTE, const char* bandMaskNB);
    bool   checkCFUN();
    bool   checkCOPS(const char* requiredOperator, const char* requiredURAT);
    bool   checkProfile(const uint8_t requiredProfile);
    bool   checkUrat(const char* requiredURAT);
    bool   checkURC(char* buffer);
    bool   doSIMcheck();
    bool   setNetworkLEDState();
    bool   isValidIPv4(const char* str);

    GSMResponseTypes readResponse(char* outBuffer = NULL, size_t outMaxSize = 0, const char* prefix = NULL,
                                  uint32_t timeout = DEFAULT_READ_MS);

    void   reboot();
    bool   setSimPin(const char* simPin);
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
