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

#ifndef _Sodaq_LteM_h
#define _Sodaq_LteM_h

#define DEFAULT_UDP_TIMOUT_MS 15000
#define MAX_UDP_BUFFER 256
#define SODAQ_LTEM_DEFAULT_CID 1

#include "Arduino.h"
#include "Sodaq_AT_Device.h"

struct SaraUDPPacketMetadata {
    uint8_t socketID;
    char ip[16]; // max IP size 4*3 digits + 3 dots + zero term = 16
    int port;
    int length;
    int remainingLength;
};

class Sodaq_LteM: public Sodaq_AT_Device
{
    public:
        Sodaq_LteM();
        
        enum TriBoolStates
        {
            TriBoolFalse,
            TriBoolTrue,
            TriBoolUndefined
        };

        enum HttpRequestTypes {
            POST = 0,
            GET,
            HEAD,
            DELETE,
            PUT,
            HttpRequestTypesMAX
        };

        typedef TriBoolStates tribool_t;

        enum SentMessageStatus {
            Pending,
            Error
        };

        struct ReceivedMessageStatus {
            uint16_t pending;
            uint16_t receivedSinceBoot;
            uint16_t droppedSinceBoot;
        };
        
        typedef ResponseTypes(*CallbackMethodPtr)(ResponseTypes& response, const char* buffer, size_t size,
                void* parameter, void* parameter2);
                
        bool setRadioActive(bool on);
        bool setApn(const char* apn);
        bool getEpoch(uint32_t* epoch);
        bool setVerboseErrors(bool on);
        
        // Returns true if the modem replies to "AT" commands without timing out.
        bool isAlive();
        
        // Returns the default baud rate of the modem.
        // To be used when initializing the modem stream for the first time.

        uint32_t getDefaultBaudrate() { return getSaraR4Baudrate(); };
        uint32_t getSaraR4Baudrate() { return 115200; };
        
        // Initializes the modem instance. Sets the modem stream and the on-off power pins.
        void init(Stream& stream, int8_t onoffPin, int8_t txEnablePin = -1, int8_t saraR4XXTogglePin = -1, uint8_t cid = SODAQ_LTEM_DEFAULT_CID);
        
        // Turns on and initializes the modem, then connects to the network and activates the data connection.
        bool connect(const char* apn, const char* forceOperator);
        
        // Disconnects the modem from the network.
        bool disconnect();

        // ==== HTTP

        // Creates an HTTP request using the (optional) given buffer and 
        // (optionally) returns the received data.
        // endpoint should include the initial "/".
        size_t httpRequest(const char* server, uint16_t port, const char* endpoint,
            HttpRequestTypes requestType = HttpRequestTypes::GET,
            char* responseBuffer = NULL, size_t responseSize = 0,
            const char* sendBuffer = NULL, size_t sendSize = 0);

        // Creates an HTTP GET request and optionally returns the received data.
        // Note. Endpoint should include the initial "/".
        // The UBlox device stores the received data in http_last_response_<profile_id>
        uint32_t httpGet(const char* server, uint16_t port, const char* endpoint,
            char* buffer, size_t bufferSize);

        // Determine HTTP header size
        uint32_t httpGetHeaderSize(const char * filename);

        // Return a partial result of the previous HTTP Request (GET or POST)
        // Offset 0 is the byte directly after the HTTP Response header
        size_t httpGetPartial(uint8_t* buffer, size_t size, uint32_t offset);
        
        // Returns true if the modem is connected to the network and has an activated data connection.
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
        size_t socketSend(uint8_t socket, const char* remoteIP, const uint16_t remotePort, const uint8_t* buffer, size_t size);
        size_t socketReceiveHex(char* buffer, size_t length, SaraUDPPacketMetadata* p = NULL);
        size_t socketReceiveBytes(uint8_t* buffer, size_t length, SaraUDPPacketMetadata* p = NULL);
        size_t getPendingUDPBytes();
        bool hasPendingUDPBytes();
        bool closeSocket(uint8_t socket);
        bool waitForUDPResponse(uint32_t timeoutMS = DEFAULT_UDP_TIMOUT_MS);

        size_t readFile(const char* filename, uint8_t* buffer, size_t size);
        size_t readFilePartial(const char* filename, uint8_t* buffer, size_t size, uint32_t offset);
        bool writeFile(const char* filename, const uint8_t* buffer, size_t size);
        bool deleteFile(const char* filename);
        bool listFiles();
        bool getRemainingFreeSpace(uint32_t & size);
        bool getFileSize(const char* filename, uint32_t & size);
        bool getIMEI(char* buffer, size_t size);
        bool getCCID(char* buffer, size_t size);
        bool getCellid(char* buffer, size_t size);
        bool getFirmwareVersion(char* buffer, size_t size);

    protected:
        // override
        ResponseTypes readResponse(char* buffer, size_t size, size_t* outSize, uint32_t timeout = SODAQ_AT_DEVICE_DEFAULT_READ_MS)
        {
            return readResponse(_inputBuffer, _inputBufferSize, NULL, NULL, NULL, outSize, timeout);
        };
        
        ResponseTypes readResponse(char* buffer, size_t size,
                                   CallbackMethodPtr parserMethod, void* callbackParameter, void* callbackParameter2 = NULL,
                                   size_t* outSize = NULL, uint32_t timeout = SODAQ_AT_DEVICE_DEFAULT_READ_MS);
                                   
        ResponseTypes readResponse(size_t* outSize = NULL, uint32_t timeout = SODAQ_AT_DEVICE_DEFAULT_READ_MS)
        {
            return readResponse(_inputBuffer, _inputBufferSize, NULL, NULL, NULL, outSize, timeout);
        };
        
        ResponseTypes readResponse(CallbackMethodPtr parserMethod, void* callbackParameter,
                                   void* callbackParameter2 = NULL, size_t* outSize = NULL, uint32_t timeout = SODAQ_AT_DEVICE_DEFAULT_READ_MS)
        {
            return readResponse(_inputBuffer, _inputBufferSize,
                                parserMethod, callbackParameter, callbackParameter2,
                                outSize, timeout);
        };
        
        template<typename T1, typename T2>
        ResponseTypes readResponse(ResponseTypes(*parserMethod)(ResponseTypes& response, const char* parseBuffer, size_t size, T1* parameter, T2* parameter2),
                                   T1* callbackParameter, T2* callbackParameter2,
                                   size_t* outSize = NULL, uint32_t timeout = SODAQ_AT_DEVICE_DEFAULT_READ_MS)
        {
            return readResponse(_inputBuffer, _inputBufferSize, (CallbackMethodPtr)parserMethod,
                                (void*)callbackParameter, (void*)callbackParameter2, outSize, timeout);
        };
        
        void purgeAllResponsesRead();
    private:

        //uint16_t _socketPendingBytes[SOCKET_COUNT]; // TODO add getter
        //bool _socketClosedBit[SOCKET_COUNT];
        
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

        // http
        tribool_t _httpRequestSuccessBit[HttpRequestTypesMAX];
        static int _httpRequestTypeToModemIndex(HttpRequestTypes requestType);
        int _httpModemIndexToRequestType(uint8_t modemIndex);
        uint32_t _httpGetHeaderSize;

        
        // flag indicating UDP response via URC
        int _receivedUDPResponseSocket = 0;
        size_t _pendingUDPBytes = 0;
        
        uint8_t _cid = 0;

        static bool startsWith(const char* pre, const char* str);
        static size_t ipToString(IP_t ip, char* buffer, size_t size);
        static bool isValidIPv4(const char* str);

        bool setR4XXToLteM();

        bool waitForSignalQuality(uint32_t timeout = 5L * 60L * 1000);
        void reboot();
        
        // For sara R4XX, receiving in chunks does NOT work, you have to receive the full packet
        size_t socketReceive(SaraUDPPacketMetadata* packet, char* buffer, size_t size);
        static uint32_t convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec);

        static ResponseTypes _copsParser(ResponseTypes& response, const char* buffer, size_t size, char* operatorNameBuffer, size_t* operatorNameBufferSize);

        static ResponseTypes _cclkParser(ResponseTypes& response, const char* buffer, size_t size, uint32_t* epoch, uint8_t* dummy);
        static ResponseTypes _csqParser(ResponseTypes& response, const char* buffer, size_t size, int* rssi, int* ber);

        static ResponseTypes _createSocketParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* socket, uint8_t* dummy);
        static ResponseTypes _udpReadSocketParser(ResponseTypes& response, const char* buffer, size_t size, SaraUDPPacketMetadata* packet, char* data);
        static ResponseTypes _sendSocketParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* socket, size_t* length);
        static ResponseTypes _udpReadURCParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* socket, size_t* length);

        static ResponseTypes _cgattParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* result, uint8_t* dummy);

        static ResponseTypes _ulstfileSizeParser(ResponseTypes& response, const char* buffer, size_t size, uint32_t* filesize, uint8_t* dummy);
        static ResponseTypes _ulstfileNamesParser(ResponseTypes& response, const char* buffer, size_t size, char* names, size_t* namesSize);

        static ResponseTypes _ccidParser(ResponseTypes& response, const char* buffer, size_t size,
                                  char* ccidBuffer, size_t* ccidBufferSize);
        static ResponseTypes _nakedStringParser(ResponseTypes& response, const char* buffer, size_t size, char* stringBuffer, size_t* stringBufferSize);

        static ResponseTypes _cellidParser(ResponseTypes& response, const char* buffer, size_t size, char* cellidBuffer, size_t* cellidBufferSize);
  };

#endif

