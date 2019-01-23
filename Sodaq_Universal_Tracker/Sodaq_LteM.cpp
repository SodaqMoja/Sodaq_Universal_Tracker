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

#include "Sodaq_LteM.h"
#include "Sodaq_wdt.h"
#include "time.h"

#define DEBUG

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000, 00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100        // years since 1900
#define COPS_TIMEOUT 180000

#define STR_AT "AT"
#define STR_RESPONSE_OK "OK"
#define STR_RESPONSE_ERROR "ERROR"
#define STR_RESPONSE_CME_ERROR "+CME ERROR:"
#define STR_RESPONSE_CMS_ERROR "+CMS ERROR:"

#define DEBUG_STR_ERROR "[ERROR]: "

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#ifdef DEBUG
#define debugPrintLn(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

#define CR '\r'

#define DEFAULT_PROFILE "0"

#define NO_IP_ADDRESS ((IP_t)0)

#define HTTP_SEND_TMP_FILENAME "http_tmp_put_0"
#define HTTP_RECEIVE_FILENAME "http_last_response_0"

#define IP_FORMAT "%d.%d.%d.%d"

#define IP_TO_TUPLE(x) (uint8_t)(((x) >> 24) & 0xFF), \
    (uint8_t)(((x) >> 16) & 0xFF), \
    (uint8_t)(((x) >> 8) & 0xFF), \
    (uint8_t)(((x) >> 0) & 0xFF)

#define TUPLE_TO_IP(o1, o2, o3, o4) ((((IP_t)o1) << 24) | (((IP_t)o2) << 16) | \
                                     (((IP_t)o3) << 8) | (((IP_t)o4) << 0))

#define SOCKET_FAIL -1

#define SOCKET_COUNT 7

#define NOW (uint32_t)millis()

typedef struct NameValuePair {
    const char* Name;
    bool Value;
} NameValuePair;

class Sodaq_ltemOnOff : public Sodaq_OnOffBee
{
    public:
        Sodaq_ltemOnOff();
        void init(int onoffPin, int8_t saraR4XXTogglePin = -1);
        void on();
        void off();
        bool isOn();
    private:
        int8_t _onoffPin;
        int8_t _saraR4XXTogglePin;
        bool _onoff_status;
};

static Sodaq_ltemOnOff sodaq_ltemOnOff;

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms)
{
    return (millis() - from) > nr_ms;
}

Sodaq_LteM::Sodaq_LteM() :
    _lastRSSI(0),
    _CSQtime(0),
    _minRSSI(-113) // dBm
{

}

// Returns true if the modem replies to "AT" commands without timing out.
bool Sodaq_LteM::isAlive()
{
    println(STR_AT);

    return (readResponse(NULL, 450) == ResponseOK);
}

// Initializes the modem instance. Sets the modem stream and the on-off power pins.
void Sodaq_LteM::init(Stream& stream, int8_t onoffPin, int8_t txEnablePin, int8_t saraR4XXTogglePin, uint8_t cid)
{
    debugPrintLn("[init] started.");

    initBuffer(); // safe to call multiple times

    setModemStream(stream);

    sodaq_ltemOnOff.init(onoffPin, saraR4XXTogglePin);
    _onoff = &sodaq_ltemOnOff;

    setTxEnablePin(txEnablePin);

    _cid = cid;
}

bool Sodaq_LteM::setRadioActive(bool on)
{
    print("AT+CFUN=");
    println(on ? "1" : "0");

    return (readResponse() == ResponseOK);
}

bool Sodaq_LteM::setVerboseErrors(bool on)
{
    print("AT+CMEE=");
    println(on ? "2" : "0"); // r4 supports verbose error messages

    return (readResponse() == ResponseOK);
}

/*!
    Read the next response from the modem

    Notice that we're collecting URC's here. And in the process we could
    be updating:
      _socketPendingBytes[] if +NSONMI/UUSORF: is seen
      _socketClosedBit[] if +UUSOCL: is seen
*/
ResponseTypes Sodaq_LteM::readResponse(char* buffer, size_t size,
                                        CallbackMethodPtr parserMethod, void* callbackParameter, void* callbackParameter2,
                                        size_t* outSize, uint32_t timeout)
{
    ResponseTypes response = ResponseNotFound;
    uint32_t from = NOW;

    do {
        // 250ms,  how many bytes at which baudrate?
        int count = readLn(buffer, size, 250);
        sodaq_wdt_reset();

        if (count > 0) {
            if (outSize) {
                *outSize = count;
            }

            if (_disableDiag && strncmp(buffer, "OK", 2) != 0) {
                _disableDiag = false;
            }

            debugPrint("[rdResp]: ");
            debugPrintLn(buffer);

            int param1, param2;
            if (sscanf(buffer, "+UFOTAS: %d,%d", &param1, &param2) == 2) { // Handle FOTA URC
                uint16_t blkRm = param1;
                uint8_t transferStatus = param2;

                debugPrint("Unsolicited: FOTA: ");
                debugPrint(blkRm);
                debugPrint(", ");
                debugPrintLn(transferStatus);

                continue;
            }
            else if (sscanf(buffer, "+UUHTTPCR: 0, %d, %d", &param1, &param2) == 2) {
                int requestType = _httpModemIndexToRequestType(static_cast<uint8_t>(param1));
                if (requestType >= 0) {
                    debugPrint("HTTP Result for request type ");
                    debugPrint(requestType);
                    debugPrint(": ");
                    debugPrintLn(param2);

                    if (param2 == 0) {
                        _httpRequestSuccessBit[requestType] = TriBoolFalse;
                    }
                    else if (param2 == 1) {
                        _httpRequestSuccessBit[requestType] = TriBoolTrue;
                    }
                }
                else {
                    // Unknown type
                }
                continue;
            }
            else if (sscanf(buffer, "+UUSORF: %d,%d", &param1, &param2) == 2){
                int socketID = param1;
                int dataLength = param2;

                debugPrint("Unsolicited: Socket ");
                debugPrint(socketID);
                debugPrint(": ");
                debugPrintLn(dataLength);
                _receivedUDPResponseSocket = socketID;
                _pendingUDPBytes = dataLength;

                continue;
            }

            if (startsWith(STR_AT, buffer)) {
                continue; // skip echoed back command
            }

            _disableDiag = false;

            if (startsWith(STR_RESPONSE_OK, buffer)) {
                return ResponseOK;
            }

            if (startsWith(STR_RESPONSE_ERROR, buffer) ||
                    startsWith(STR_RESPONSE_CME_ERROR, buffer) ||
                    startsWith(STR_RESPONSE_CMS_ERROR, buffer)) {
                return ResponseError;
            }

            if (parserMethod) {
                ResponseTypes parserResponse = parserMethod(response, buffer, count, callbackParameter, callbackParameter2);

                if ((parserResponse != ResponseEmpty) && (parserResponse != ResponsePendingExtra)) {
                    return parserResponse;
                }
                else {
                    // ?
                    // ResponseEmpty indicates that the parser was satisfied
                    // Continue until "OK", "ERROR", or whatever else.
                }

                // Prevent calling the parser again.
                // This could happen if the input line is too long. It will be split
                // and the next readLn will return the next part.
                // The case of "ResponsePendingExtra" is an exception to this, thus waiting for more replies to be parsed.
                if (parserResponse != ResponsePendingExtra) {
                    parserMethod = 0;
                }
            }

            // at this point, the parserMethod has ran and there is no override response from it,
            // so if there is some other response recorded, return that
            // (otherwise continue iterations until timeout)
            if (response != ResponseNotFound) {
                debugPrintLn("** response != ResponseNotFound");
                return response;
            }
        }

        delay(10);      // TODO Why do we need this delay?
    }
    while (!is_timedout(from, timeout));

    if (outSize) {
        *outSize = 0;
    }

    debugPrintLn("[rdResp]: timed out");
    return ResponseTimeout;
}

// Gets International Mobile Equipment Identity.
// Should be provided with a buffer of at least 16 bytes.
// Returns true if successful.
bool Sodaq_LteM::getIMEI(char* buffer, size_t size)
{
    if (size < 15 + 1) {
        return false;
    }

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CGSN");

    return (readResponse<char, size_t>(_nakedStringParser, buffer, &size) == ResponseOK);
}

// Gets Cell identifier
// Should be provided with a buffer of at least 9 bytes.
// Returns true if successful.
bool Sodaq_LteM::getCellid(char* buffer, size_t size)
{
    if (size < 8 + 1) {
        return false;
    }

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CREG?");

    return (readResponse<char, size_t>(_cellidParser, buffer, &size) == ResponseOK);
}

bool Sodaq_LteM::getFirmwareVersion(char * buffer, size_t size)
{
    if (size < 30 + 1) {
        return false;
    }

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CGMR");

    return (readResponse<char, size_t>(_nakedStringParser, buffer, &size) == ResponseOK);
}


ResponseTypes Sodaq_LteM::_ccidParser(ResponseTypes& response, const char* buffer, size_t size,
        char* ccidBuffer, size_t* ccidBufferSize)
{
    if (!ccidBuffer || !ccidBufferSize) {
        return ResponseError;
    }

    // TODO limit?
    if (sscanf(buffer, "+CCID: %s", ccidBuffer) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// Gets Integrated Circuit Card ID.
// Should be provided with a buffer of at least 21 bytes.
// Returns true if successful.
bool Sodaq_LteM::getCCID(char* buffer, size_t size)
{
    if (size < 20 + 1) {
        return false;
    }

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CCID");

    return (readResponse<char, size_t>(_ccidParser, buffer, &size) == ResponseOK);
}

ResponseTypes Sodaq_LteM::_nakedStringParser(ResponseTypes& response, const char* buffer,
    size_t size, char* stringBuffer, size_t* stringBufferSize)
{
    if (!stringBuffer || !stringBufferSize) {
        return ResponseError;
    }

    stringBuffer[0] = 0;
    if (*stringBufferSize > 0)
    {
        strncat(stringBuffer, buffer, *stringBufferSize - 1);

        return ResponseEmpty;
    }

    return ResponseError;
}

/**
* A convenience wrapper function to just a simple HTTP GET
*
* If the request is handled properly it will return the contents
* without the header.
* The header is checked for presence of "HTTP/1.1 200 OK" ????
* The header is skipped, that is, upto and including two CRLF's,
* the remainder is returned in the buffer.
*/
uint32_t Sodaq_LteM::httpGet(const char* server, uint16_t port, const char* endpoint,
    char* buffer, size_t bufferSize)
{
    uint32_t file_size;

    // First just handle the request and let the file be read into the UBlox file system
    file_size = httpRequest(server, port, endpoint, GET, 0, 0);
    if (file_size == 0) {
        return 0;
    }

    // Find out the header size
    _httpGetHeaderSize = httpGetHeaderSize(HTTP_RECEIVE_FILENAME);
    debugPrintLn(String("[httpGet] header size: ") + _httpGetHeaderSize);
    if (_httpGetHeaderSize == 0) {
        return 0;
    }

    if (!buffer) {
        return file_size - _httpGetHeaderSize;
    }

    // Fill the buffer starting from the header
    return readFilePartial(HTTP_RECEIVE_FILENAME, (uint8_t *)buffer, bufferSize, _httpGetHeaderSize);
}

/**
* Return the size of the HTTP response header
*
* This function searches from the start of the file until two CRLFs have
* been seen.
* The file is left unmodified.
*/
uint32_t Sodaq_LteM::httpGetHeaderSize(const char * filename)
{
    bool status;

    uint32_t file_size;
    status = getFileSize(filename, file_size);
    if (!status) {
        return 0;
    }

    int state = 0;      // 0 nothing, 1=CR, 2=CRLF, 3=CRLFCR, 4=CRLFCRLF
    uint8_t buffer[64];
    uint32_t offset = 0;
    while (offset < file_size && state != 4) {
        size_t len = sizeof(buffer);
        size_t size;
        size = readFilePartial(filename, buffer, sizeof(buffer), offset);

        size_t ix;
        for (ix = 0; state != 4 && ix < sizeof(buffer); ix++) {
            if ((state == 0 || state == 2) && buffer[ix] == '\r') {
                state++;
            }
            else if ((state == 1 || state == 3) && buffer[ix] == '\n') {
                state++;
            }
            else {
                state = 0;
            }
        }
        if (state == 4) {
            return offset + ix;
        }

        offset += size;
    }

    return 0;
}

size_t Sodaq_LteM::httpGetPartial(uint8_t* buffer, size_t size, uint32_t offset)
{
    if (_httpGetHeaderSize == 0) {
        _httpGetHeaderSize = httpGetHeaderSize(HTTP_RECEIVE_FILENAME);
    }
    if (_httpGetHeaderSize == 0) {
        // Don't trust a file without HTTP response header
        return 0;
    }
    return readFilePartial(HTTP_RECEIVE_FILENAME, buffer, size, _httpGetHeaderSize + offset);
}

// HttpRequestTypes if successful, -1 if not
int Sodaq_LteM::_httpModemIndexToRequestType(uint8_t modemIndex)
{
    static uint8_t mapping[] = {
        HEAD,   // 0
        GET,    // 1
        DELETE, // 2
        PUT,    // 3
        POST,   // 4
    };

    return (modemIndex < sizeof(mapping)) ? mapping[modemIndex] : -1;
}

// maps the given requestType to the index the modem recognizes, -1 if error
int Sodaq_LteM::_httpRequestTypeToModemIndex(HttpRequestTypes requestType)
{
    static uint8_t mapping[] = {
        4,      // 0 POST
        1,      // 1 GET
        0,      // 2 HEAD
        2,      // 3 DELETE
        3,      // 4 PUT
    };

    return (requestType < sizeof(mapping)) ? mapping[requestType] : -1;
}

bool Sodaq_LteM::setApn(const char* apn)
{
    print("AT+CGDCONT=");
    print(_cid);
    print(",\"IP\",\"");
    print(apn);
    println("\"");

    return (readResponse() == ResponseOK);
}

bool Sodaq_LteM::getEpoch(uint32_t* epoch)
{
    println("AT+CCLK?");

    return readResponse<uint32_t, uint8_t>(_cclkParser, epoch, NULL) == ResponseOK;
}

bool Sodaq_LteM::setR4XXToLteM()
{
    println("AT+URAT=7");

    return (readResponse() == ResponseOK);
}

void Sodaq_LteM::purgeAllResponsesRead()
{
    uint32_t start = millis();

    // make sure all the responses within the timeout have been read
    while ((readResponse(0, 1000) != ResponseTimeout) && !is_timedout(start, 2000)) {}
}

// Turns on and initializes the modem, then connects to the network and activates the data connection.
bool Sodaq_LteM::connect(const char* apn, const char* forceOperator)
{
    if (!on()) {
        return false;
    }

    purgeAllResponsesRead();

    println("ATE0"); // echo off
    if (readResponse() != ResponseOK) {
        debugPrintLn("Error: Failed to turn off echo")
    }

    if (!setVerboseErrors(true)) {
        return false;
    }

    setR4XXToLteM();

    // set data transfer to hex mode
    println("AT+UDCONF=1,1");
    readResponse();

#ifdef DEBUG
    println("AT+URAT?");
    readResponse();
#endif

    if (!setApn(apn)) {
        return false;
    }

    if (!setRadioActive(true)) {
        return false;
    }

    uint32_t start = millis();

    // The default timeout is 4 minutes. This may seem high, but ...
    // In some situations (new SIM card) it can take a long time
    // the get the registration. Subsequent registrations should
    // go much quicker.
    uint32_t delay_count = 500;
    bool copsSuccess = false;
    while (!is_timedout(start, COPS_TIMEOUT)) {
        sodaq_wdt_safe_delay(delay_count);
        // Next time wait a little longer
        delay_count += 1000;

        print("AT+COPS=1,2");
        if (forceOperator) {
            print(",\"");
            print(forceOperator);
            print("\"");
        }
        println();

        if (readResponse(NULL, 40000) == ResponseOK) {
            copsSuccess = true;
            break;
        }
    }

    if (!copsSuccess) {
        return false;
    }

    if (!waitForSignalQuality()) {
        return false;
    }

    // If we got this far we succeeded
    return true;
}

// Parse the result from AT+COPS? and when we want to see the operator name.
// The manual says to expect:
//   +COPS: <mode>[,<format>,<oper>[,<AcT>]]
//   OK
ResponseTypes Sodaq_LteM::_copsParser(ResponseTypes& response, const char* buffer, size_t size,
    char* operatorNameBuffer, size_t* operatorNameBufferSize)
{
    if (!operatorNameBuffer || !operatorNameBufferSize) {
        return ResponseError;
    }

    // TODO maybe limit length of string in format? needs converting int to string though
    if (sscanf(buffer, "+COPS: %*d,%*d,\"%[^\"]\",%*d", operatorNameBuffer) == 1) {
        return ResponseEmpty;
    }
    if (sscanf(buffer, "+COPS: %*d,%*d,\"%[^\"]\"", operatorNameBuffer) == 1) {
        return ResponseEmpty;
    }
    int dummy;
    if (sscanf(buffer, "+COPS: %d", &dummy) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// Parse the result from AT+CREG? and when we want to see the operator name.
// The manual says to expect:
//   +CREG: <n>,<stat>[,<lac>,<ci>[,<AcTStatus>]]
//   OK
ResponseTypes Sodaq_LteM::_cellidParser(ResponseTypes& response, const char* buffer, size_t size, char* cellidBuffer, size_t* cellidBufferSize)
{
    if (!cellidBuffer || !cellidBufferSize) {
        return ResponseError;
    }

    int dummy1;
    int dummy2;

    // TODO maybe limit length of string in format? needs converting int to string though
    if (sscanf(buffer, "+CREG: %*d,%*d,%*s,\"%[^\"]\",%*d", cellidBuffer) == 1) {
        return ResponseEmpty;
    }
    else if (sscanf(buffer, "+CREG: %*d,%*d,%*s,\"%[^\"]\"", cellidBuffer) == 1) {
        return ResponseEmpty;
    }
    else if (sscanf(buffer, "+CREG: %d,%d", &dummy1, &dummy2) == 2) {
        return ResponseEmpty;
    }

    return ResponseError;
}

void Sodaq_LteM::reboot()
{
    println("AT+CFUN=15"); // reset modem + sim

    // wait up to 2000ms for the modem to come up
    uint32_t start = millis();

    while ((readResponse() != ResponseOK) && !is_timedout(start, 2000)) { }
}

int Sodaq_LteM::createSocket(uint16_t localPort)
{
    print("AT+USOCR=17,");
    println(localPort);

    uint8_t socket;

    if (readResponse<uint8_t, uint8_t>(_createSocketParser, &socket, NULL) == ResponseOK) {
        return socket;
    }

    return SOCKET_FAIL;
}

bool Sodaq_LteM::closeSocket(uint8_t socketID)
{
    // only Datagram/UDP is supported
    print("AT+USOCL=");
    println(socketID);

    return readResponse() == ResponseOK;
}

size_t Sodaq_LteM::socketSend(uint8_t socket, const char* remoteIP, const uint16_t remotePort, const uint8_t* buffer, size_t size)
{
    if (size > 512) {
        debugPrintLn("SocketSend exceeded maximum buffer size!");
        return -1;
    }

    // only Datagram/UDP is supported
    print("AT+USOST=");
    print(socket);
    print(',');
    print('\"');
    print(remoteIP);
    print('\"');
    print(',');
    print(remotePort);
    print(',');
    print(size);
    print(',');
    print('\"');
    for (uint16_t i = 0; i < size; ++i) {
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(buffer[i]))));
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(buffer[i]))));
    }
    println('\"');

    uint8_t retSocketID;
    size_t sentLength;

    if (readResponse<uint8_t, size_t>(_sendSocketParser, &retSocketID, &sentLength, 0, 10 * 1000) == ResponseOK) {
        return sentLength;
    }

    return 0;
}

bool Sodaq_LteM::waitForUDPResponse(uint32_t timeoutMS)
{
    if (hasPendingUDPBytes()) { return true; }

    uint32_t startTime = millis();

    while (!hasPendingUDPBytes() && (millis() - startTime) < timeoutMS) {
        print("AT+USORF=");
        print(0);
        print(",");
        println(0);

        uint8_t socketID = 0;
        size_t length = 0;

        if (readResponse<uint8_t, size_t>(_udpReadURCParser, &socketID, &length) == ResponseOK) {
            _receivedUDPResponseSocket = socketID;
            _pendingUDPBytes = length;
        }
        sodaq_wdt_safe_delay(10);
    }

    return hasPendingUDPBytes();
}

size_t Sodaq_LteM::getPendingUDPBytes()
{
    return _pendingUDPBytes;
}

bool Sodaq_LteM::hasPendingUDPBytes()
{
    return _pendingUDPBytes > 0;
}

size_t Sodaq_LteM::socketReceive(SaraUDPPacketMetadata* packet, char* buffer, size_t size)
{
    if (_pendingUDPBytes == 0) {
        // no URC has happened, no socket to read
        debugPrintLn("Reading from without available bytes!");
        return 0;
    }

    print("AT+USORF=");
    print(_receivedUDPResponseSocket);
    print(',');
    println(size);

    if (readResponse<SaraUDPPacketMetadata, char>(_udpReadSocketParser, packet, buffer) == ResponseOK) {
        // update pending bytes
        _pendingUDPBytes -= packet->length;
        return packet->length;
    }

    debugPrintLn("Reading from socket failed!");
    return 0;
}

size_t Sodaq_LteM::socketReceiveHex(char* buffer, size_t length, SaraUDPPacketMetadata* p)
{
    SaraUDPPacketMetadata packet;

    size_t receiveSize = length;

    receiveSize = min(receiveSize, _pendingUDPBytes);
    return socketReceive(p ? p : &packet, buffer, receiveSize);
}

size_t Sodaq_LteM::socketReceiveBytes(uint8_t* buffer, size_t length, SaraUDPPacketMetadata* p)
{
    size_t size = min(length, min(MAX_UDP_BUFFER, _pendingUDPBytes));

    SaraUDPPacketMetadata packet;
    char tempBuffer[MAX_UDP_BUFFER];

    size_t receivedSize = socketReceive(p ? p : &packet, tempBuffer, size);

    if (buffer && length > 0) {
        for (size_t i = 0; i < receivedSize * 2; i += 2) {
            buffer[i / 2] = HEX_PAIR_TO_BYTE(tempBuffer[i], tempBuffer[i + 1]);
        }
    }

    return receivedSize;
}

ResponseTypes Sodaq_LteM::_createSocketParser(ResponseTypes& response, const char* buffer, size_t size,
        uint8_t* socket, uint8_t* dummy)
{
    if (!socket) {
        return ResponseError;
    }

    int socketID;

    if (sscanf(buffer, "%d", &socketID) == 1) {
        if (socketID <= UINT8_MAX) {
            *socket = socketID;
        }
        else {
            return ResponseError;
        }

        return ResponseEmpty;
    }

    if (sscanf(buffer, "+USOCR: %d", &socketID) == 1) {
        if (socketID <= UINT8_MAX) {
            *socket = socketID;
        }
        else {
            return ResponseError;
        }

        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_LteM::_sendSocketParser(ResponseTypes& response, const char* buffer, size_t size,
        uint8_t* socket, size_t* length)
{
    if (!socket) {
        return ResponseError;
    }

    int socketID;
    int sendSize;

    if (sscanf(buffer, "%d,%d", &socketID, &sendSize) == 2) {
        if (socketID <= UINT8_MAX) {
            *socket = socketID;
        }
        else {
            return ResponseError;
        }

        if (socketID <= SIZE_MAX) {
            *length = sendSize;
        }
        else {
            return ResponseError;
        }

        return ResponseEmpty;
    }

    if (sscanf(buffer, "+USOST: %d,%d", &socketID, &sendSize) == 2) {
        if (socketID <= UINT8_MAX) {
            *socket = socketID;
        }
        else {
            return ResponseError;
        }

        if (socketID <= SIZE_MAX) {
            *length = sendSize;
        }
        else {
            return ResponseError;
        }

        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_LteM::_udpReadSocketParser(ResponseTypes& response, const char* buffer, size_t size, SaraUDPPacketMetadata* packet, char* data)
{
    // fixes bad behavior from the module, size == 1 is a sanity check to prevent future bugs passing silently
    if ( (size == 1) && (buffer[0] == CR) ) {
        return ResponsePendingExtra;
    }

    if (!packet) {
        return ResponseError;
    }

    int socketID;

    if (sscanf(buffer, "%d,\"%[^\"]\",%d,%d,\"%[^\"]\",%d", &socketID, packet->ip, &packet->port, &packet->length, data, &packet->remainingLength) == 6) {
        if (socketID <= UINT8_MAX) {
            packet->socketID = socketID;
        }
        else {
            return ResponseError;
        }
        return ResponseEmpty;
    }

    if (sscanf(buffer, "+USORF: %d,\"%[^\"]\",%d,%d,\"%[^\"]\"", &socketID, packet->ip, &packet->port, &packet->length, data) == 5) {
        if (socketID <= UINT8_MAX) {
            packet->socketID = socketID;
        }
        else {
            return ResponseError;
        }
        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_LteM::_udpReadURCParser(ResponseTypes& response, const char* buffer, size_t size,
    uint8_t* socket, size_t* length)
{
    int socketID;
    int receiveSize;

    // fixes bad behavior from the module, size == 1 is a sanity check to prevent future bugs passing silently
    if ((size == 1) && (buffer[0] == CR)) {
        return ResponsePendingExtra;
    }

    if (sscanf(buffer, "+USORF: %d,%d", &socketID, &receiveSize) == 2) {
        if (socketID <= UINT8_MAX) {
            *socket = socketID;
        }
        else {
            return ResponseError;
        }

        if (socketID <= SIZE_MAX) {
            *length = receiveSize;
        }
        else {
            return ResponseError;
        }

        return ResponseEmpty;
    }

    return ResponseError;
}

bool connectSocket(uint8_t socket, const char* host, uint16_t port)
{
    return false;
}

// Creates an HTTP request using the (optional) given buffer and
// (optionally) returns the received data.
// endpoint should include the initial "/".
size_t Sodaq_LteM::httpRequest(const char* server, uint16_t port,
    const char* endpoint, HttpRequestTypes requestType,
    char* responseBuffer, size_t responseSize,
    const char* sendBuffer, size_t sendSize)
{
    // TODO maybe return error <0 ?

    // reset http profile 0
    println("AT+UHTTP=0");
    if (readResponse() != ResponseOK) {
        return 0;
    }

    //println("AT+UHTTP=0,6,1");
    //if (readResponse() != ResponseOK) {
    //    debugPrintLn("Could not set secure param!");
    //    return 0;
    //}

    deleteFile(HTTP_RECEIVE_FILENAME); // cleanup the file first (if exists)

    if (requestType >= HttpRequestTypesMAX) {
        debugPrintLn(DEBUG_STR_ERROR "Unknown request type!");
        return 0;
    }

    // set server host name
    print("AT+UHTTP=0,");
    print(isValidIPv4(server) ? "0,\"" : "1,\"");
    print(server);
    println("\"");
    if (readResponse() != ResponseOK) {
        return 0;
    }

    // set port
    if (port != 80) {
        print("AT+UHTTP=0,5,");
        println(port);

        if (readResponse() != ResponseOK) {
            return 0;
        }
    }

    // before starting the actual http request, create any files needed in the fs of the modem
    // that way there is a chance to abort sending the http req command in case of an fs error
    if (requestType == PUT || requestType == POST) {
        if (!sendBuffer || sendSize == 0) {
            debugPrintLn(DEBUG_STR_ERROR "There is no sendBuffer or sendSize set!");
            return 0;
        }

        deleteFile(HTTP_SEND_TMP_FILENAME); // cleanup the file first (if exists)

        if (!writeFile(HTTP_SEND_TMP_FILENAME, (uint8_t*)sendBuffer, sendSize)) {
            debugPrintLn(DEBUG_STR_ERROR "Could not create the http tmp file!");
            return 0;
        }
    }

    // reset the success bit before calling a new request
    _httpRequestSuccessBit[requestType] = TriBoolUndefined;

    print("AT+UHTTPC=0,");
    print(_httpRequestTypeToModemIndex(requestType));
    print(",\"");
    print(endpoint);
    print("\",\"\""); // empty filename = default = "http_last_response_0" (DEFAULT_HTTP_RECEIVE_FILENAME)

                      // NOTE: a file that includes the buffer to send has been created already
    if (requestType == PUT) {
        print(",\"" HTTP_SEND_TMP_FILENAME "\""); // param1: file from filesystem to send
    }
    else if (requestType == POST) {
        print(",\"" HTTP_SEND_TMP_FILENAME "\""); // param1: file from filesystem to send
        print(",1"); // param2: content type, 1=text/plain
                     // TODO consider making the content type a parameter
    }
    else {
        // GET, etc
    }
    println("");

    if (readResponse() != ResponseOK) {
        return 0;
    }

    // check for success while checking URCs
    // This loop relies on readResponse being called via isAlive()
    uint32_t start = millis();
    uint32_t delay_count = 50;
    while ((_httpRequestSuccessBit[requestType] == TriBoolUndefined) && !is_timedout(start, /*TODO*/5000)) {
        isAlive();
        sodaq_wdt_safe_delay(delay_count);
        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 250;
        }
    }

    if (_httpRequestSuccessBit[requestType] == TriBoolTrue) {
        uint32_t file_size;
        if (!getFileSize(HTTP_RECEIVE_FILENAME, file_size)) {
            debugPrintLn(DEBUG_STR_ERROR "Could not determine file size");
            return 0;
        }
        if (responseBuffer && responseSize > 0 && file_size < responseSize) {
            return readFile(HTTP_RECEIVE_FILENAME, (uint8_t*)responseBuffer, responseSize);
        }
        else {
            // On AVR this can give size_t overflow
            return file_size;
        }
    }
    else if (_httpRequestSuccessBit[requestType] == TriBoolFalse) {
        debugPrintLn(DEBUG_STR_ERROR "An error occurred with the http request!");
        return 0;
    }
    else {
        debugPrintLn(DEBUG_STR_ERROR "Timed out waiting for a response for the http request!");
        return 0;
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////    UBLOX FILE UTILITIES   /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


/**
* Read a file from the UBlox device
*/
size_t Sodaq_LteM::readFile(const char* filename, uint8_t* buffer, size_t size)
{
    // TODO escape filename characters { '"', ',', }

    //sanity check
    if (!buffer || size == 0) {
        return 0;
    }

    // first, make sure the buffer is sufficient
    uint32_t filesize = 0;
    if (!getFileSize(filename, filesize)) {
        debugPrintLn(DEBUG_STR_ERROR "Could not determine file size");
        return 0;
    }

    if (filesize > size) {
        debugPrintLn(DEBUG_STR_ERROR "The buffer is not big enough to store the file");
        return 0;
    }

    print("AT+URDFILE=\"");
    print(filename);
    println("\"");

    // override normal parsing process and explicitly read characters here
    // to be able to also read terminator characters within files
    char checkChar = 0;
    size_t len = 0;

    // reply identifier
    len = readBytesUntil(' ', _inputBuffer, _inputBufferSize);
    if (len == 0 || strstr(_inputBuffer, "+URDFILE:") == NULL) {
        debugPrintLn(DEBUG_STR_ERROR "+URDFILE literal is missing!");
        goto error;
    }

    // filename
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    // TODO check filename after removing quotes and escaping chars
    //if (len == 0 || strstr(_inputBuffer, filename)) {
    //    debugPrintLn(DEBUG_STR_ERROR "Filename reported back is not correct!");
    //    return 0;
    //}

    // filesize
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    filesize = 0; // reset the var before reading from reply string
    if (sscanf(_inputBuffer, "%lu", &filesize) != 1) {
        debugPrintLn(DEBUG_STR_ERROR "Could not parse the file size!");
        goto error;
    }
    if (filesize == 0 || filesize > size) {
        debugPrintLn(DEBUG_STR_ERROR "Size error!");
        goto error;
    }

    // opening quote character
    checkChar = timedRead();
    if (checkChar != '"') {
        debugPrintLn(DEBUG_STR_ERROR "Missing starting character (quote)!");
        goto error;
    }

    // actual file buffer, written directly to the provided result buffer
    len = readBytes(buffer, filesize);
    if (len != filesize) {
        debugPrintLn(DEBUG_STR_ERROR "File size error!");
        goto error;
    }

    // closing quote character
    checkChar = timedRead();
    if (checkChar != '"') {
        debugPrintLn(DEBUG_STR_ERROR "Missing termination character (quote)!");
        goto error;
    }

    // read final OK response from modem and return the filesize
    if (readResponse() == ResponseOK) {
        return filesize;
    }

error:
    return 0;
}

/**
* Read a file from the UBlox device
*/
size_t Sodaq_LteM::readFilePartial(const char* filename, uint8_t* buffer, size_t size, uint32_t offset)
{
    // TODO escape filename characters { '"', ',', }

    //sanity check
    if (!buffer || size == 0) {
        return 0;
    }

#if 0
    // Probably no need to read the file size as that should have been done by the caller
    uint32_t filesize = 0;
    if (!getFileSize(filename, filesize)) {
        debugPrintLn(DEBUG_STR_ERROR "Could not determine file size");
        return 0;
    }

    if (filesize > size) {
        debugPrintLn(DEBUG_STR_ERROR "The buffer is not big enough to store the file");
        return 0;
    }
#endif

    print("AT+URDBLOCK=\"");
    print(filename);
    print("\",");
    print(offset);
    print(",");
    println(size);

    // override normal parsing process and explicitly read characters here
    // to be able to also read terminator characters within files

    char quote;
    size_t len = 0;
    uint32_t blocksize;

    // reply identifier
    //   +URDBLOCK: http_last_response_0,86,"..."
    // where 86 is an example of the size
    len = readBytesUntil(' ', _inputBuffer, _inputBufferSize);
    if (len == 0 || strstr(_inputBuffer, "+URDBLOCK:") == NULL) {
        debugPrintLn(DEBUG_STR_ERROR "+URDBLOCK literal is missing!");
        goto error;
    }

    // skip filename
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    // TODO check filename. Note, there are no quotes

    // read the number of bytes
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    blocksize = 0; // reset the var before reading from reply string
    if (sscanf(_inputBuffer, "%lu", &blocksize) != 1) {
        debugPrintLn(DEBUG_STR_ERROR "Could not parse the block size!");
        goto error;
    }
    if (blocksize == 0 || blocksize > size) {
        debugPrintLn(DEBUG_STR_ERROR "Size error!");
        goto error;
    }

    // opening quote character
    quote = timedRead();
    if (quote != '"') {
        debugPrintLn(DEBUG_STR_ERROR "Missing starting character (quote)!");
        goto error;
    }

    // actual file buffer, written directly to the provided result buffer
    len = readBytes(buffer, blocksize);
    if (len != blocksize) {
        debugPrintLn(DEBUG_STR_ERROR "File size error!");
        goto error;
    }

    // closing quote character
    quote = timedRead();
    if (quote != '"') {
        debugPrintLn(DEBUG_STR_ERROR "Missing termination character (quote)!");
        goto error;
    }

    // read final OK response from modem and return the filesize
    if (readResponse() == ResponseOK) {
        return blocksize;
    }

error:
    return 0;
}

bool Sodaq_LteM::writeFile(const char* filename, const uint8_t* buffer, size_t size)
{
    // TODO escape filename characters
    print("AT+UDWNFILE=\"");
    print(filename);
    print("\",");
    println(size);

    if (readResponse() == ResponsePrompt) {
        for (size_t i = 0; i < size; i++) {
            print(buffer[i]);
        }

        return readResponse() == ResponseOK;
    }

    return false;
}

bool Sodaq_LteM::deleteFile(const char* filename)
{
    // TODO escape filename characters
    print("AT+UDELFILE=\"");
    print(filename);
    println("\"");

    return readResponse() == ResponseOK;
}

bool Sodaq_LteM::listFiles()
{
    // list all files
    println("AT+ULSTFILE=0");

    // First parse +ULSTFILE: "file1","file2" ...
    char names[30];
    size_t names_size = sizeof(names);

    return readResponse<char, size_t>(_ulstfileNamesParser, names, &names_size) == ResponseOK;
}

ResponseTypes Sodaq_LteM::_ulstfileNamesParser(ResponseTypes& response, const char* buffer, size_t size,
    char* names, size_t* namesSize)
{
    if (!names || !namesSize) {
        return ResponseError;
    }

    const char *prefix = "+ULSTFILE: ";
    size_t prefix_len = strlen(prefix);
    if (strncmp(buffer, prefix, prefix_len) == 0) {
        memset(names, 0, *namesSize);
        strncpy(names, buffer, *namesSize - 1);
        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_LteM::_ulstfileSizeParser(ResponseTypes& response, const char* buffer, size_t size,
    uint32_t* filesize, uint8_t* dummy)
{
    if (!filesize) {
        return ResponseError;
    }

    if (sscanf(buffer, "+ULSTFILE: %lu", filesize) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

bool Sodaq_LteM::getRemainingFreeSpace(uint32_t & size)
{
    // get free space
    println("AT+ULSTFILE=1");

    return readResponse<uint32_t, uint8_t>(_ulstfileSizeParser, &size, NULL) == ResponseOK;
}

bool Sodaq_LteM::getFileSize(const char* filename, uint32_t & size)
{
    print("AT+ULSTFILE=2,\"");
    print(filename);
    println("\"");

    // If the file is not present you'll get:
    //   +CME ERROR: FILE NOT FOUND
    return readResponse<uint32_t, uint8_t>(_ulstfileSizeParser, &size, NULL) == ResponseOK;
}

// Disconnects the modem from the network.
bool Sodaq_LteM::disconnect()
{
    println("AT+COPS=2");

    return (readResponse(NULL, 40000) == ResponseOK);
}

// Returns true if the modem is connected to the network and has an activated data connection.
bool Sodaq_LteM::isConnected()
{
    uint8_t value = 0;

    println("AT+CGATT?");

    if (readResponse<uint8_t, uint8_t>(_cgattParser, &value, NULL) == ResponseOK) {
        return (value == 1);
    }

    return false;
}

// Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
// Returns true if successful.
bool Sodaq_LteM::getRSSIAndBER(int8_t* rssi, uint8_t* ber)
{
    static char berValues[] = { 49, 43, 37, 25, 19, 13, 7, 0 }; // 3GPP TS 45.008 [20] subclause 8.2.4

    println("AT+CSQ");

    int csqRaw = 0;
    int berRaw = 0;

    if (readResponse<int, int>(_csqParser, &csqRaw, &berRaw) == ResponseOK) {
        *rssi = ((csqRaw == 99) ? 0 : convertCSQ2RSSI(csqRaw));
        *ber = ((berRaw == 99 || static_cast<size_t>(berRaw) >= sizeof(berValues)) ? 0 : berValues[berRaw]);

        return true;
    }

    return false;
}

/*
    The range is the following:
    0: -113 dBm or less
    1: -111 dBm
    2..30: from -109 to -53 dBm with 2 dBm steps
    31: -51 dBm or greater
    99: not known or not detectable or currently not available
*/
int8_t Sodaq_LteM::convertCSQ2RSSI(uint8_t csq) const
{
    return -113 + 2 * csq;
}

uint8_t Sodaq_LteM::convertRSSI2CSQ(int8_t rssi) const
{
    return (rssi + 113) / 2;
}

bool Sodaq_LteM::startsWith(const char* pre, const char* str)
{
    return (strncmp(pre, str, strlen(pre)) == 0);
}

size_t Sodaq_LteM::ipToString(IP_t ip, char* buffer, size_t size)
{
    return snprintf(buffer, size, IP_FORMAT, IP_TO_TUPLE(ip));
}

bool Sodaq_LteM::isValidIPv4(const char* str)
{
    uint8_t segs = 0; // Segment count
    uint8_t chcnt = 0; // Character count within segment
    uint8_t accum = 0; // Accumulator for segment

    if (!str) {
        return false;
    }

    // Process every character in string
    while (*str != '\0') {
        // Segment changeover
        if (*str == '.') {
            // Must have some digits in segment
            if (chcnt == 0) {
                return false;
            }

            // Limit number of segments
            if (++segs == 4) {
                return false;
            }

            // Reset segment values and restart loop
            chcnt = accum = 0;
            str++;
            continue;
        }

        // Check numeric
        if ((*str < '0') || (*str > '9')) {
            return false;
        }

        // Accumulate and check segment
        if ((accum = accum * 10 + *str - '0') > 255) {
            return false;
        }

        // Advance other segment specific stuff and continue loop
        chcnt++;
        str++;
    }

    // Check enough segments and enough characters in last segment
    if (segs != 3) {
        return false;
    }

    if (chcnt == 0) {
        return false;
    }

    // Address OK

    return true;
}

bool Sodaq_LteM::waitForSignalQuality(uint32_t timeout)
{
    uint32_t start = millis();
    const int8_t minRSSI = getMinRSSI();
    int8_t rssi;
    uint8_t ber;

    uint32_t delay_count = 500;

    while (!is_timedout(start, timeout)) {
        if (getRSSIAndBER(&rssi, &ber)) {
            if (rssi != 0 && rssi >= minRSSI) {
                _lastRSSI = rssi;
                _CSQtime = (int32_t)(millis() - start) / 1000;
                return true;
            }
        }

        sodaq_wdt_safe_delay(delay_count);

        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 1000;
        }
    }

    return false;
}

ResponseTypes Sodaq_LteM::_cgattParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* result, uint8_t* dummy)
{
    if (!result) {
        return ResponseError;
    }

    int val;

    if (sscanf(buffer, "+CGATT: %d", &val) == 1) {
        *result = val;
        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_LteM::_csqParser(ResponseTypes& response, const char* buffer, size_t size,
                                      int* rssi, int* ber)
{
    if (!rssi || !ber) {
        return ResponseError;
    }

    if (sscanf(buffer, "+CSQ: %d,%d", rssi, ber) == 2) {
        return ResponseEmpty;
    }

    return ResponseError;
}

uint32_t Sodaq_LteM::convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec)
{
    struct tm tm;

    tm.tm_isdst = -1;
    tm.tm_yday = 0;
    tm.tm_wday = 0;
    tm.tm_year = y + EPOCH_TIME_YEAR_OFF;
    tm.tm_mon = m - 1;
    tm.tm_mday = d;
    tm.tm_hour = h;
    tm.tm_min = min;
    tm.tm_sec = sec;

    return mktime(&tm);
}

ResponseTypes Sodaq_LteM::_cclkParser(ResponseTypes& response, const char* buffer, size_t size,
                                       uint32_t* epoch, uint8_t* dummy)
{
    if (!epoch) {
        return ResponseError;
    }

    // format: "yy/MM/dd,hh:mm:ss+TZ
    int y, m, d, h, min, sec, tz;
    if (sscanf(buffer, "+CCLK: \"%d/%d/%d,%d:%d:%d+%d\"", &y, &m, &d, &h, &min, &sec, &tz) == 7) {
        *epoch = convertDatetimeToEpoch(y, m, d, h, min, sec);
        return ResponseEmpty;
    }
    else if (sscanf(buffer, "+CCLK: \"%d/%d/%d,%d:%d:%d\"", &y, &m, &d, &h, &min, &sec) == 6) {
        *epoch = convertDatetimeToEpoch(y, m, d, h, min, sec);
        return ResponseEmpty;
    }

    return ResponseError;
}

// ==============================
// on/off class
// ==============================
Sodaq_ltemOnOff::Sodaq_ltemOnOff()
{
    _onoffPin = -1;
    _onoff_status = false;
}

// Initializes the instance
void Sodaq_ltemOnOff::init(int onoffPin, int8_t saraR4XXTogglePin)
{
    if (onoffPin >= 0) {
        _onoffPin = onoffPin;
        // First write the output value, and only then set the output mode.
        digitalWrite(_onoffPin, LOW);
        pinMode(_onoffPin, OUTPUT);
    }

    // always set this because its optional and can be -1
    _saraR4XXTogglePin = saraR4XXTogglePin;
    if (saraR4XXTogglePin >= 0) {
        pinMode(_saraR4XXTogglePin, OUTPUT);
    }
}

void Sodaq_ltemOnOff::on()
{
    if (_onoffPin >= 0) {
        digitalWrite(_onoffPin, HIGH);
    }

    if (_saraR4XXTogglePin >= 0) {
        pinMode(_saraR4XXTogglePin, OUTPUT);
        digitalWrite(_saraR4XXTogglePin, LOW);
        sodaq_wdt_safe_delay(2000);
        pinMode(_saraR4XXTogglePin, INPUT);
    }

    _onoff_status = true;
}

void Sodaq_ltemOnOff::off()
{
    // The GPRSbee is switched off immediately
    if (_onoffPin >= 0) {
        digitalWrite(_onoffPin, LOW);
    }

    // Should be instant
    // Let's wait a little, but not too long
    delay(50);
    _onoff_status = false;
}

bool Sodaq_ltemOnOff::isOn()
{
    #if defined(ARDUINO_ARCH_AVR)
    // Use the onoff pin, which is close to useless
    bool status = digitalRead(_onoffPin);
    return status;
    #elif defined(ARDUINO_ARCH_SAMD)
    // There is no status pin. On SAMD we cannot read back the onoff pin.
    // So, our own status is all we have.
    return _onoff_status;
    #endif

    // Let's assume it is on.
    return true;
}
