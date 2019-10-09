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

#include "Sodaq_R4X.h"
#include "Sodaq_wdt.h"
#include "time.h"

#define DEBUG

#define EPOCH_TIME_OFF             946684800  /* This is 1st January 2000, 00:00:00 in epoch time */
#define EPOCH_TIME_YEAR_OFF        100        /* years since 1900 */
#define ATTACH_TIMEOUT             180000
#define ATTACH_NEED_REBOOT         40000
#define CGACT_TIMEOUT              150000
#define COPS_TIMEOUT               180000
#define ISCONNECTED_CSQ_TIMEOUT    10000
#define REBOOT_DELAY               1250
#define REBOOT_TIMEOUT             15000
#define SOCKET_CLOSE_TIMEOUT       120000
#define SOCKET_CONNECT_TIMEOUT     120000
#define SOCKET_WRITE_TIMEOUT       120000
#define UMQTT_TIMEOUT              60000
#define POWER_OFF_DELAY            5000

#define SODAQ_GSM_TERMINATOR "\r\n"
#define SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE 1024
#define SODAQ_GSM_TERMINATOR_LEN (sizeof(SODAQ_GSM_TERMINATOR) - 1)

#define NETWORK_STATUS_GPIO_ID     16

#define DEBUG_STR_ERROR            "[ERROR]: "
#define STR_AT                     "AT"
#define STR_RESPONSE_OK            "OK"
#define STR_RESPONSE_ERROR         "ERROR"
#define STR_RESPONSE_CME_ERROR     "+CME ERROR:"
#define STR_RESPONSE_CMS_ERROR     "+CMS ERROR:"
#define STR_RESPONSE_SOCKET_PROMPT '@'
#define STR_RESPONSE_FILE_PROMPT   '>'

#define NIBBLE_TO_HEX_CHAR(i)  ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i)         ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i)          (i & 0x0F)
#define HEX_CHAR_TO_NIBBLE(c)  ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define HTTP_RECEIVE_FILENAME  "http_last_response_0"
#define HTTP_SEND_TMP_FILENAME "http_tmp_put_0"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifdef DEBUG
#define debugPrint(...)   { if (_diagStream) _diagStream->print(__VA_ARGS__); }
#define debugPrintln(...) { if (_diagStream) _diagStream->println(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrint(...)
#define debugPrintln(...)
#endif

#define CR '\r'
#define LF '\n'

#define SOCKET_FAIL -1
#define NOW (uint32_t)millis()

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms) { return (millis() - from) > nr_ms; }

static uint8_t httpRequestMapping[] = {
    4, // 0 POST
    1, // 1 GET
    0, // 2 HEAD
    2, // 3 DELETE
    3, // 4 PUT
};


/******************************************************************************
* Main
*****************************************************************************/

Sodaq_R4X::Sodaq_R4X() :
    _modemStream(0),
    _diagStream(0),
    _appendCommand(false),
    _CSQtime(0),
    _startOn(0)
{
    _isBufferInitialized = false;
    _inputBuffer         = 0;
    _inputBufferSize     = SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE;
    _lastRSSI            = 0;
    _minRSSI             = -113;  // dBm
    _onoff               = 0;
    _mqttLoginResult     = -1;
    _mqttPendingMessages = -1;
    _mqttSubscribeReason = -1;
    _networkStatusLED = 0;
    _pin                 = 0;

    memset(_socketClosedBit,    1, sizeof(_socketClosedBit));
    memset(_socketPendingBytes, 0, sizeof(_socketPendingBytes));
}

// Initializes the modem instance. Sets the modem stream and the on-off power pins.
void Sodaq_R4X::init(Sodaq_OnOffBee* onoff, Stream& stream, uint8_t cid)
{
    debugPrintln("[init] started.");

    initBuffer(); // safe to call multiple times

    setModemStream(stream);

    _onoff = onoff;
    _cid   = cid;
}

// Turns the modem on and returns true if successful.
bool Sodaq_R4X::on()
{
    _startOn = millis();

    if (!isOn() && _onoff) {
        _onoff->on();
    }

    // wait for power up
    bool timeout = true;
    for (uint8_t i = 0; i < 10; i++) {
        if (isAlive()) {
            timeout = false;
            break;
        }
    }

    if (timeout) {
        debugPrintln("Error: No Reply from Modem");
        return false;
    }

    // Extra read just to clear the input stream
    readResponse(NULL, 0, NULL, 250);

    return isOn(); // this essentially means isOn() && isAlive()
}

// Turns the modem off and returns true if successful.
bool Sodaq_R4X::off()
{
    // Safety command to shutdown, response is ignored
    if (isOn()) {
        println("AT+CPWROFF");
        readResponse(NULL, 0, NULL, 1000);
        sodaq_wdt_safe_delay(POWER_OFF_DELAY);
    }

    // No matter if it is on or off, turn it off.
    if (_onoff) {
        _onoff->off();
    }

    _mqttLoginResult = -1;

    return !isOn();
}

bool Sodaq_R4X::connect(const char* apn, const char* uratSelect, uint8_t mnoProfile,
    const char* operatorSelect, const char* bandMaskLTE, const char* bandMaskNB)
{
    if (!on()) {
        return false;
    }

    purgeAllResponsesRead();

    if (!setVerboseErrors(true)) {
        return false;
    }

    if (!setNetworkLEDState()) {
        return false;
    }

    if (!execCommand("ATE0")) {
        return false;
    }

    if (!checkCFUN()) {
        return false;
    }

    if (!checkProfile(mnoProfile)) {
        return false;
    }

    if (!checkUrat(uratSelect != 0 ? uratSelect : DEFAULT_URAT)) {
        return false;
    }

    if (!checkBandMasks(bandMaskLTE, bandMaskNB)) {
        return false;
    }

    if (!checkCOPS(operatorSelect != 0 ? operatorSelect : AUTOMATIC_OPERATOR, uratSelect != 0 ? uratSelect : DEFAULT_URAT)) {
        return false;
    }

    int8_t i = checkApn(apn);
    if (i < 0) {
        return false;
    }

    uint32_t tm = millis();

    if (!waitForSignalQuality()) {
        return false;
    }

    if (i == 0 && !attachGprs(ATTACH_TIMEOUT)) {
        return false;
    }

    if (millis() - tm > ATTACH_NEED_REBOOT) {
        reboot();
        
        if (!waitForSignalQuality()) {
            return false;
        }

        if (!attachGprs(ATTACH_TIMEOUT)) {
            return false;
        }
    }

    return doSIMcheck();
}

bool Sodaq_R4X::connect(const char* apn, const char* urat, const char* bandMask)
{
    connect(apn, urat, SIM_ICCID, AUTOMATIC_OPERATOR, BAND_MASK_UNCHANGED, bandMask);
}

// Disconnects the modem from the network.
bool Sodaq_R4X::disconnect()
{
    return execCommand("AT+CGATT=0", 40000);
}


/******************************************************************************
* Public
*****************************************************************************/

bool Sodaq_R4X::attachGprs(uint32_t timeout)
{
    uint32_t start = millis();
    uint32_t delay_count = 500;

    while (!is_timedout(start, timeout)) {
        if (isAttached()) {
            if (isDefinedIP4() || (execCommand("AT+CGACT=1", CGACT_TIMEOUT) && isDefinedIP4())) {
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

bool Sodaq_R4X::bandMasktoStr(const uint64_t bandMask, char* str, size_t size)
{
    bool result = false;

    uint8_t i = 0;
    uint64_t n = bandMask;

    do {
        i++;
    } while (n /= 10);

    if (i < size) {
        str[i] = '\0';
        n = bandMask;

        do {
            str[--i] = (n % 10) + '0';
        } while (n /= 10);

        result = true;
    }

    return result;
}

// Gets Integrated Circuit Card ID.
// Should be provided with a buffer of at least 21 bytes.
// Returns true if successful.
bool Sodaq_R4X::getCCID(char* buffer, size_t size)
{
    if (buffer == NULL || size < 20 + 1) {
        return false;
    }

    println("AT+CCID");

    return (readResponse(buffer, size, "+CCID: ") == GSMResponseOK) && (strlen(buffer) > 0);
}

// Gets the International Mobile Subscriber Identity
// Should be provided with a buffer of at least 21 bytes.
// Returns true if successful.
bool Sodaq_R4X::getIMSI(char* buffer, size_t size)
{
    if (buffer == NULL || size < 20 + 1) {
        return false;
    }

    return (execCommand("AT+CIMI", DEFAULT_READ_MS, buffer, size) == GSMResponseOK) && (strlen(buffer) > 0) && (atoll(buffer) > 0);
}

bool Sodaq_R4X::getOperatorInfo(uint16_t* mcc, uint16_t* mnc)
{
    println("AT+COPS=3,2");

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    println("AT+COPS?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    uint32_t operatorCode = 0;

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+COPS: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {

        if (sscanf(responseBuffer, "%*d,%*d,\"%u\"", &operatorCode) == 1) {
            uint16_t divider = (operatorCode > 100000) ? 1000 : 100;

            *mcc = operatorCode / divider;
            *mnc = operatorCode % divider;

            return true;
        }
    }

    return false;
}

bool Sodaq_R4X::getOperatorInfoString(char* buffer, size_t size)
{
    if (size < 32 + 1) {
         return false;
    }

    buffer[0] = 0;

    println("AT+COPS=3,0");

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    println("AT+COPS?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+COPS: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {

        if (sscanf(responseBuffer, "%*d,%*d,\"%[^\"]\"", buffer) == 1) {
            return true;
        }
    }

    return false;
}

bool Sodaq_R4X::getCellInfo(uint16_t* tac, uint32_t* cid, uint16_t* urat)
{
    println("AT+CEREG=2");

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    println("AT+CEREG?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+CEREG: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {
        if (sscanf(responseBuffer, "2,%*d,\"%hx\",\"%x\",%hi", tac, cid, urat) == 3) {
            switch(*urat) {
                case 7:
                case 8: *urat = 7; break;
                case 9: *urat = 8; break;
            }
            return true;
        }
        else {
            // if +CEREG did not return the tac/cid/act
            // lets try +CGREG for GPRS registration info
            println("AT+CGREG=2");

            if (readResponse() != GSMResponseOK) {
                return false;
            }

            println("AT+CGREG?");

            memset(responseBuffer, 0, sizeof(responseBuffer));

            if ((readResponse(responseBuffer, sizeof(responseBuffer), "+CGREG: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {
                if (sscanf(responseBuffer, "2,%*d,\"%hx\",\"%x\"", tac, cid) == 2) {
                    *urat = 9;
                    return true;
                }
            }
        }
    }

    return false;
}

bool Sodaq_R4X::getEpoch(uint32_t* epoch)
{
    println("AT+CCLK?");

    char buffer[128];

    if (readResponse(buffer, sizeof(buffer), "+CCLK: ") != GSMResponseOK) {
        return false;
    }

    // format: "yy/MM/dd,hh:mm:ss+TZ
    int y, m, d, h, min, sec, tz;
    if (sscanf(buffer, "\"%d/%d/%d,%d:%d:%d+%d\"", &y, &m, &d, &h, &min, &sec, &tz) == 7 ||
        sscanf(buffer, "\"%d/%d/%d,%d:%d:%d\"", &y, &m, &d, &h, &min, &sec) == 6)
    {
        *epoch = convertDatetimeToEpoch(y, m, d, h, min, sec);
        return true;
    }

    return false;
}

bool Sodaq_R4X::getFirmwareVersion(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("AT+CGMR", DEFAULT_READ_MS, buffer, size);
}

bool Sodaq_R4X::getFirmwareRevision(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("ATI9", DEFAULT_READ_MS, buffer, size);
}

// Gets International Mobile Equipment Identity.
// Should be provided with a buffer of at least 16 bytes.
// Returns true if successful.
bool Sodaq_R4X::getIMEI(char* buffer, size_t size)
{
    if (buffer == NULL || size < 15 + 1) {
        return false;
    }

    return (execCommand("AT+CGSN", DEFAULT_READ_MS, buffer, size) == GSMResponseOK) && (strlen(buffer) > 0) && (atoll(buffer) > 0);
}

SimStatuses Sodaq_R4X::getSimStatus()
{
    println("AT+CPIN?");

    char buffer[32];

    if (readResponse(buffer, sizeof(buffer)) != GSMResponseOK) {
        return SimStatusUnknown;
    }

    char status[16];

    if (sscanf(buffer, "+CPIN: %" STR(sizeof(status) - 1) "s", status) == 1) {
        return startsWith("READY", status) ? SimReady : SimNeedsPin;
    }

    return SimMissing;
}

bool Sodaq_R4X::execCommand(const char* command, uint32_t timeout, char* buffer, size_t size)
{
    println(command);

    return (readResponse(buffer, size, NULL, timeout) == GSMResponseOK);
}

// Returns true if the modem replies to "AT" commands without timing out.
bool Sodaq_R4X::isAlive()
{
    return execCommand(STR_AT, 450);
}

// Returns true if the modem is attached to the network and has an activated data connection.
bool Sodaq_R4X::isAttached()
{
    println("AT+CGATT?");

    char buffer[16];

    if (readResponse(buffer, sizeof(buffer), "+CGATT: ", 10 * 1000) != GSMResponseOK) {
        return false;
    }

    return (strcmp(buffer, "1") == 0);
}

// Returns true if the modem is connected to the network and IP address is not 0.0.0.0.
bool Sodaq_R4X::isConnected()
{
    return isOn() && isAttached() && waitForSignalQuality(ISCONNECTED_CSQ_TIMEOUT) && isDefinedIP4() ;
}

// Returns true if defined IP4 address is not 0.0.0.0.
bool Sodaq_R4X::isDefinedIP4()
{
    println("AT+CGDCONT?");

    char buffer[256];

    if (readResponse(buffer, sizeof(buffer), "+CGDCONT: ") != GSMResponseOK) {
        return false;
    }

    if (strncmp(buffer, "1,\"IP\"", 6) != 0 || strncmp(buffer + 6, ",\"\"", 3) == 0) {
        return false;
    }

    char ip[32];

    if (sscanf(buffer + 6, ",\"%*[^\"]\",\"%[^\"]\",0,0,0,0", ip) != 1) {
        return false;
    }

    return strlen(ip) >= 7 && strcmp(ip, "0.0.0.0") != 0;
}

void Sodaq_R4X::purgeAllResponsesRead()
{
    uint32_t start = millis();

    // make sure all the responses within the timeout have been read
    while ((readResponse(NULL, 0, NULL, 1000) != GSMResponseTimeout) && !is_timedout(start, 2000)) {}
}

bool Sodaq_R4X::setApn(const char* apn)
{
    print("AT+CGDCONT=");
    print(_cid);
    print(",\"IP\",\"");
    print(apn);
    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::setIndicationsActive(bool on)
{
    print("AT+CNMI=");
    println(on ? "1" : "0");

    return (readResponse() == GSMResponseOK);
}

void Sodaq_R4X::setPin(const char * pin)
{
    size_t len = strlen(pin);
    _pin = static_cast<char*>(realloc(_pin, len + 1));
    strcpy(_pin, pin);
}

bool Sodaq_R4X::setRadioActive(bool on)
{
    print("AT+CFUN=");
    println(on ? "1" : "0");

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::setVerboseErrors(bool on)
{
    print("AT+CMEE=");
    println(on ? '2' : '0');

    return (readResponse() == GSMResponseOK);
}


/******************************************************************************
* RSSI and CSQ
*****************************************************************************/

/*
    The range is the following:
    0: -113 dBm or less
    1: -111 dBm
    2..30: from -109 to -53 dBm with 2 dBm steps
    31: -51 dBm or greater
    99: not known or not detectable or currently not available
*/
int8_t Sodaq_R4X::convertCSQ2RSSI(uint8_t csq) const
{
    return -113 + 2 * csq;
}

uint8_t Sodaq_R4X::convertRSSI2CSQ(int8_t rssi) const
{
    return (rssi + 113) / 2;
}

// Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
// Returns true if successful.
bool Sodaq_R4X::getRSSIAndBER(int8_t* rssi, uint8_t* ber)
{
    static char berValues[] = { 49, 43, 37, 25, 19, 13, 7, 0 }; // 3GPP TS 45.008 [20] subclause 8.2.4

    println("AT+CSQ");

    char buffer[256];

    if (readResponse(buffer, sizeof(buffer), "+CSQ: ") != GSMResponseOK) {
        return false;
    }

    int csqRaw;
    int berRaw;

    if (sscanf(buffer, "%d,%d", &csqRaw, &berRaw) != 2) {
        return false;
    }

    *rssi = ((csqRaw == 99) ? 0 : convertCSQ2RSSI(csqRaw));
    *ber  = ((berRaw == 99 || static_cast<size_t>(berRaw) >= sizeof(berValues)) ? 0 : berValues[berRaw]);

    return true;
}


/******************************************************************************
* Sockets
*****************************************************************************/

bool Sodaq_R4X::socketClose(uint8_t socketID, bool async)
{
    socketFlush(socketID);
    
    print("AT+USOCL=");
    print(socketID);

    if (async) {
        println(",1");
    }
    else {
        println();
    }

    _socketClosedBit   [socketID] = true;
    _socketPendingBytes[socketID] = 0;

    if (readResponse(NULL, 0, NULL, SOCKET_CLOSE_TIMEOUT) != GSMResponseOK) {
        return false;
    }

    return true;
}

int Sodaq_R4X::socketCloseAll() {

    int closedCount = 0;

    for (uint8_t i = 0; i < SOCKET_COUNT; i++) {
        if (socketClose(i, false)) {
            closedCount++;
        }
    }

    // return the number of sockets we closed
    return closedCount;
}

bool Sodaq_R4X::socketConnect(uint8_t socketID, const char* remoteHost, const uint16_t remotePort)
{
    print("AT+USOCO=");
    print(socketID);
    print(",\"");
    print(remoteHost);
    print("\",");
    println(remotePort);

    bool b = readResponse(NULL, 0, NULL, SOCKET_CONNECT_TIMEOUT) == GSMResponseOK;

    _socketClosedBit  [socketID] = !b;

    return b;
}

int Sodaq_R4X::socketCreate(uint16_t localPort, Protocols protocol)
{
    print("AT+USOCR=");
    print(protocol == UDP ? "17" : "6");

    if (localPort > 0) {
        print(',');
        println(localPort);
    }
    else {
        println();
    }

    char buffer[32];

    if (readResponse(buffer, sizeof(buffer), "+USOCR: ") != GSMResponseOK) {
        return SOCKET_FAIL;
    }

    int socketID;

    if ((sscanf(buffer, "%d", &socketID) != 1) || (socketID < 0) || (socketID > SOCKET_COUNT)) {
        return SOCKET_FAIL;
    }

    _socketClosedBit   [socketID] = true;
    _socketPendingBytes[socketID] = 0;

    return socketID;
}

bool Sodaq_R4X::socketFlush(uint8_t socketID, uint32_t timeout)
{
    uint32_t start = millis();

    while (isAlive() && (!is_timedout(start, timeout)) && (!_socketClosedBit[socketID])) {
        print("AT+USOCTL=");
        print(socketID);
        println(",11");
        
        char buffer[32];

        if (readResponse(buffer, sizeof(buffer), "+USOCTL: ") != GSMResponseOK) {
            if (_socketClosedBit[socketID]) {
                return true;
            }
            else {
                return false;
            }
        }
        
        int pendingBytes = 0;
        if (sscanf(buffer, "%*d,11,%d", &pendingBytes) != 1) {
            return false;
        }
        else if (pendingBytes == 0){
            return true;
        }
        
        sodaq_wdt_safe_delay(300);
    }
    
    if (_socketClosedBit[socketID]) {
        return true;
    }

    return false;
}

size_t Sodaq_R4X::socketGetPendingBytes(uint8_t socketID)
{
    return _socketPendingBytes[socketID];
}

bool Sodaq_R4X::socketHasPendingBytes(uint8_t socketID)
{
    return socketGetPendingBytes(socketID) > 0;
}

bool Sodaq_R4X::socketIsClosed(uint8_t socketID)
{
    return _socketClosedBit[socketID];
}

size_t Sodaq_R4X::socketRead(uint8_t socketID, uint8_t* buffer, size_t size)
{
    if (!socketHasPendingBytes(socketID)) {
        // no URC has happened, no socket to read
        debugPrintln("Reading from without available bytes!");
        return 0;
    }

    execCommand("AT+UDCONF=1,1");

    size = min(size, min(SODAQ_R4X_MAX_SOCKET_BUFFER, _socketPendingBytes[socketID]));

    char   outBuffer[SODAQ_R4X_MAX_SOCKET_BUFFER];
    int    retSocketID;
    size_t retSize;

    print("AT+USORD=");
    print(socketID);
    print(',');
    println(size);

    if (readResponse(outBuffer, sizeof(outBuffer), "+USORD: ") != GSMResponseOK) {
        return 0;
    }

    if (sscanf(outBuffer, "%d,%d,\"%[^\"]\"", &retSocketID, &retSize, outBuffer) != 3) {
        return 0;
    }

    if ((retSocketID < 0) || (retSocketID >= SOCKET_COUNT)) {
        return 0;
    }

    _socketPendingBytes[socketID] -= retSize;

    if (buffer != NULL && size > 0) {
        for (size_t i = 0; i < retSize * 2; i += 2) {
            buffer[i / 2] = HEX_PAIR_TO_BYTE(outBuffer[i], outBuffer[i + 1]);
        }
    }

    return retSize;
}

size_t Sodaq_R4X::socketReceive(uint8_t socketID, uint8_t* buffer, size_t size)
{
    if (!socketHasPendingBytes(socketID)) {
        // no URC has happened, no socket to read
        debugPrintln("Reading from without available bytes!");
        return 0;
    }

    execCommand("AT+UDCONF=1,1");

    size = min(size, min(SODAQ_R4X_MAX_SOCKET_BUFFER, _socketPendingBytes[socketID]));

    char   outBuffer[SODAQ_R4X_MAX_SOCKET_BUFFER];
    int    retSocketID;
    size_t retSize;

    print("AT+USORF=");
    print(socketID);
    print(',');
    println(size);

    if (readResponse(outBuffer, sizeof(outBuffer), "+USORF: ") != GSMResponseOK) {
        return 0;
    }

    if (sscanf(outBuffer, "%d,\"%*[^\"]\",%*d,%d,\"%[^\"]\"", &retSocketID, &retSize, outBuffer) != 3) {
        return 0;
    }

    if ((retSocketID < 0) || (retSocketID >= SOCKET_COUNT)) {
        return 0;
    }

    _socketPendingBytes[socketID] -= retSize;

    if (buffer != NULL && size > 0) {
        for (size_t i = 0; i < retSize * 2; i += 2) {
            buffer[i / 2] = HEX_PAIR_TO_BYTE(outBuffer[i], outBuffer[i + 1]);
        }
    }

    return retSize;
}

size_t Sodaq_R4X::socketSend(uint8_t socketID, const char* remoteHost, const uint16_t remotePort,
                             const uint8_t* buffer, size_t size)
{
    if (size > SODAQ_MAX_SEND_MESSAGE_SIZE) {
        debugPrintln("Message exceeded maximum size!");
        return 0;
    }

    execCommand("AT+UDCONF=1,1");

    print("AT+USOST=");
    print(socketID);
    print(",\"");
    print(remoteHost);
    print("\",");
    print(remotePort);
    print(',');
    print(size);
    print(",\"");

    for (size_t i = 0; i < size; ++i) {
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(buffer[i]))));
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(buffer[i]))));
    }

    println('"');

    char outBuffer[64];

    if (readResponse(outBuffer, sizeof(outBuffer), "+USOST: ", SOCKET_WRITE_TIMEOUT) != GSMResponseOK) {
        return 0;
    }

    int retSocketID;
    int sentLength;

    if ((sscanf(outBuffer, "%d,%d", &retSocketID, &sentLength) != 2) || (retSocketID < 0) || (retSocketID > SOCKET_COUNT)) {
        return 0;
    }

    return sentLength;
}

bool Sodaq_R4X::socketSetR4KeepAlive(uint8_t socketID)
{
    return socketSetR4Option(socketID, 65535, 8, 1);
}

bool Sodaq_R4X::socketSetR4Option(uint8_t socketID, uint16_t level, uint16_t optName, uint32_t optValue, uint32_t optValue2)
{
    print("AT+USOSO=");
    print(socketID);
    print(',');
    print(level);
    print(',');
    print(optName);
    print(',');
    print(optValue);

    if (optValue2 > 0) {
        print(',');
        println(optValue2);
    }
    else {
        println();
    }

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::socketWaitForClose(uint8_t socketID, uint32_t timeout)
{
    uint32_t startTime = millis();

    while (isAlive() && !socketIsClosed(socketID) && !is_timedout(startTime, timeout)) {
        sodaq_wdt_safe_delay(10);
    }

    return socketIsClosed(socketID);
}

bool Sodaq_R4X::socketWaitForRead(uint8_t socketID, uint32_t timeout)
{
    if (socketHasPendingBytes(socketID)) {
        return true;
    }

    uint32_t startTime = millis();

    while (!socketHasPendingBytes(socketID) && !socketIsClosed(socketID) && !is_timedout(startTime, timeout)) {
        print("AT+USORD=");
        print(socketID);
        println(",0");

        char buffer[128];

        if (readResponse(buffer, sizeof(buffer), "+USORD: ") == GSMResponseOK) {
            int retSocketID;
            int receiveSize;

            if (sscanf(buffer, "%d,%d", &retSocketID, &receiveSize) == 2) {
                _socketPendingBytes[retSocketID] = receiveSize;
            }
        }

        sodaq_wdt_safe_delay(10);
    }

    return socketHasPendingBytes(socketID);
}

bool Sodaq_R4X::socketWaitForReceive(uint8_t socketID, uint32_t timeout)
{
    if (socketHasPendingBytes(socketID)) {
        return true;
    }

    uint32_t startTime = millis();

    while (!socketHasPendingBytes(socketID) && !is_timedout(startTime, timeout)) {
        print("AT+USORF=");
        print(socketID);
        println(",0");

        char buffer[128];

        if (readResponse(buffer, sizeof(buffer), "+USORF: ") == GSMResponseOK) {
            int retSocketID;
            int receiveSize;

            if (sscanf(buffer, "%d,%d", &retSocketID, &receiveSize) == 2) {
                _socketPendingBytes[retSocketID] = receiveSize;
            }
        }

        sodaq_wdt_safe_delay(250);
    }

    return socketHasPendingBytes(socketID);
}

size_t Sodaq_R4X::socketWrite(uint8_t socketID, const uint8_t* buffer, size_t size)
{
    execCommand("AT+UDCONF=1,1");

    print("AT+USOWR=");
    print(socketID);
    print(",");
    println(size);

    if (readResponse() != GSMResponsePrompt) {
        return 0;
    }

    // After the @ prompt reception, wait for a minimum of 50 ms before sending data.
    delay(51);

    for (size_t i = 0; i < size; i++) {
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(buffer[i]))));
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(buffer[i]))));
    }

    debugPrintln();

    char outBuffer[64];

    if (readResponse(outBuffer, sizeof(outBuffer), "+USOWR: ", SOCKET_WRITE_TIMEOUT) != GSMResponseOK) {
        return 0;
    }

    int retSocketID;
    int sentLength;

    if ((sscanf(outBuffer, "%d,%d", &retSocketID, &sentLength) != 2) || (retSocketID < 0) || (retSocketID > SOCKET_COUNT)) {
        return 0;
    }

    return sentLength;
}


/******************************************************************************
* MQTT
*****************************************************************************/

int8_t Sodaq_R4X::mqttGetLoginResult()
{
    return _mqttLoginResult;
}

int16_t Sodaq_R4X::mqttGetPendingMessages()
{
    return _mqttPendingMessages;
}

bool Sodaq_R4X::mqttLogin(uint32_t timeout)
{
    char buffer[16];

    _mqttLoginResult = -1;

    println("AT+UMQTTC=1");

    uint32_t startTime = millis();

    if ((readResponse(buffer, sizeof(buffer), "+UMQTTC: ", timeout) != GSMResponseOK) || !startsWith("1,1", buffer)) {
        return false;
    }

    // check URC synchronously
    while ((_mqttLoginResult == -1) && !is_timedout(startTime, timeout)) {
        mqttLoop();
    }

    return (_mqttLoginResult == 0);
}

bool Sodaq_R4X::mqttLogout()
{
    char buffer[16];
    _mqttLoginResult = -1;

    println("AT+UMQTTC=0");

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("0,1", buffer);
}

void Sodaq_R4X::mqttLoop()
{
    sodaq_wdt_reset();
    if (!_modemStream->available()) {
        return;
    }

    int count = readLn(_inputBuffer, _inputBufferSize, 250); // 250ms, how many bytes at which baudrate?
    sodaq_wdt_reset();

    if (count <= 0) {
        return;
    }

    debugPrint("<< ");
    debugPrintln(_inputBuffer);

    checkURC(_inputBuffer);
}

bool Sodaq_R4X::mqttPing(const char* server)
{
    char buffer[16];

    print("AT+UMQTTC=8,\"");
    print(server);
    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("8,1", buffer);
}

bool Sodaq_R4X::mqttPublish(const char* topic, const uint8_t* msg, size_t size, uint8_t qos, uint8_t retain, bool useHEX)
{
    char buffer[16];

    print("AT+UMQTTC=2,");
    print(qos);
    print(',');
    print(retain);
    if (useHEX) {
        print(",1");
    }
    print(",\"");
    print(topic);
    print("\",\"");

    for (size_t i = 0; i < size; ++i) {
        if (useHEX) {
            print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(msg[i]))));
            print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(msg[i]))));
        }
        else {
            print((char)msg[i]);
        }
    }

    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("2,1", buffer);
}

// returns number of read messages
uint16_t Sodaq_R4X::mqttReadMessages(char* buffer, size_t size, uint32_t timeout)
{
    if (buffer == NULL || size == 0) {
        return 0;
    }

    char bufferIn[16];

    _mqttPendingMessages = -1;

    println("AT+UMQTTC=6");

    uint32_t startTime = millis();

    if ((readResponse(bufferIn, sizeof(bufferIn), "+UMQTTC: ", UMQTT_TIMEOUT) != GSMResponseOK) || !startsWith("6,1", bufferIn)) {
        return 0;
    }

    // check URC synchronously
    while ((_mqttPendingMessages == -1) && !is_timedout(startTime, timeout)) {
        mqttLoop();
    }

    if (_mqttPendingMessages <= 0) {
        return 0;
    }

    uint16_t messages = 0;
    uint16_t outSize  = 0;
    char* topicStart = 0;
    char* messageStart = 0;

    while (messages < _mqttPendingMessages && !is_timedout(startTime, timeout)) {
        int count = readLn(_inputBuffer, _inputBufferSize, 250);
        sodaq_wdt_reset();

        if (count <= 0) {
            continue;
        }

        bool isTopic = startsWith("Topic:", _inputBuffer);
        bool isMessage = startsWith("Msg:", _inputBuffer);

        if (isTopic || isMessage) {
            // init publish msg vars when seeing a topic (topic comes first)
            if (isTopic) {
                topicStart = 0;
                messageStart = 0;

                outSize = 0;
            }

            if (outSize >= size - 1) {
                break;
            }

            uint8_t headerSize = isMessage ? strlen("Msg:") : strlen("Topic:");
            count -= headerSize;
            if (isTopic) {
                count--; // remove the LF at the end
            }

            if (outSize + (uint16_t)count > size - 1) {
                count = size - 1 - outSize;
            }

            memcpy(buffer + outSize, _inputBuffer + headerSize, count);
            if (isTopic) {
                topicStart = buffer + outSize;
            }
            else { // isMessage
                messageStart = buffer + outSize;
            }

            outSize += count;
            buffer[outSize++] = 0;

            // if there is already a full set of topic+message read, call the handler
            if ((topicStart != 0) && (messageStart != 0)) {
                messages++;

                if (_mqttPublishHandler) {
                    _mqttPublishHandler(topicStart, messageStart);
                }
            }

        }
    }

    _mqttPendingMessages -= messages;

    return messages;
}

bool Sodaq_R4X::mqttSetAuth(const char* name, const char* pw)
{
    char buffer[16];

    print("AT+UMQTT=4,\"");
    print(name);
    print("\",\"");
    print(pw);
    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("4,1", buffer);
}

bool Sodaq_R4X::mqttSetCleanSession(bool enabled)
{
    char buffer[16];

    print("AT+UMQTT=12,");
    println(enabled ? '1' : '0');

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("12,1", buffer);
}

bool Sodaq_R4X::mqttSetClientId(const char* id)
{
    char buffer[16];

    print("AT+UMQTT=0,\"");
    print(id);
    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("0,1", buffer);
}

bool Sodaq_R4X::mqttSetInactivityTimeout(uint16_t timeout)
{
    char buffer[16];

    print("AT+UMQTT=10,");
    println(timeout);

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("10,1", buffer);
}

bool Sodaq_R4X::mqttSetLocalPort(uint16_t port)
{
    char buffer[16];

    print("AT+UMQTT=1,");
    println(port);

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("1,1", buffer);
}

bool Sodaq_R4X::mqttSetSecureOption(bool enabled, int8_t profile)
{
    char buffer[16];

    print("AT+UMQTT=11,");
    print(enabled ? '1' : '0');

    if (profile >= 0) {
        print(',');
        println(profile);
    }

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("11,1", buffer);
}

bool Sodaq_R4X::mqttSetServer(const char* server, uint16_t port)
{
    char buffer[16];

    print("AT+UMQTT=2,\"");
    print(server);

    if (port > 0) {
        print("\",");
        println(port);
    }
    else {
        println('"');
    }

  return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("2,1", buffer);
}

bool Sodaq_R4X::mqttSetServerIP(const char* ip, uint16_t port)
{
    char buffer[16];

    print("AT+UMQTT=3,\"");
    print(ip);

    if (port > 0) {
        print("\",");
        println(port);
    }
    else {
        println('"');
    }

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("3,1", buffer);
}

bool Sodaq_R4X::mqttSubscribe(const char* filter, uint8_t qos, uint32_t timeout)
{
    char buffer[16];

    _mqttSubscribeReason = -1;

    print("AT+UMQTTC=4,");
    print(qos);
    print(",\"");
    print(filter);
    println('"');

    uint32_t startTime = millis();

    if ((readResponse(buffer, sizeof(buffer), "+UMQTTC: ", UMQTT_TIMEOUT) != GSMResponseOK) || !startsWith("4,1", buffer)) {
        return false;
    }

    // check URC synchronously
    while ((_mqttSubscribeReason == -1) && !is_timedout(startTime, timeout)) {
        mqttLoop();
    }

    return (_mqttSubscribeReason == 1);
}

bool Sodaq_R4X::mqttUnsubscribe(const char* filter)
{
    char buffer[16];

    print("AT+UMQTTC=5,\"");
    print(filter);
    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", UMQTT_TIMEOUT) == GSMResponseOK) && startsWith("5,1", buffer);
}

void Sodaq_R4X::mqttSetPublishHandler(PublishHandlerPtr handler)
{
    if (handler) {
        _mqttPublishHandler = handler;
    }
}


/******************************************************************************
* HTTP
*****************************************************************************/

/**
* A convenience wrapper function to just a simple HTTP GET
*
* If the request is handled properly it will return the contents
* without the header.
* The header is checked for presence of "HTTP/1.1 200 OK" ????
* The header is skipped, that is, upto and including two CRLF's,
* the remainder is returned in the buffer.
*/
uint32_t Sodaq_R4X::httpGet(const char* server, uint16_t port, const char* endpoint,
                            char* buffer, size_t bufferSize, uint32_t timeout, bool useURC)
{
    // First just handle the request and let the file be read into the UBlox file system
    uint32_t file_size = httpRequest(server, port, endpoint, GET, NULL, 0, NULL, 0, timeout, useURC);
    if (file_size == 0) {
        return 0;
    }

    // Find out the header size
    _httpGetHeaderSize = httpGetHeaderSize(HTTP_RECEIVE_FILENAME);

    debugPrint("[httpGet] header size: ");
    debugPrintln(_httpGetHeaderSize);

    if (_httpGetHeaderSize == 0) {
        return 0;
    }

    if (!buffer) {
        return file_size - _httpGetHeaderSize;
    }

    // Fill the buffer starting from the header
    return readFilePartial(HTTP_RECEIVE_FILENAME, (uint8_t*)buffer, bufferSize, _httpGetHeaderSize);
}

/**
* Return the size of the HTTP response header
*
* This function searches from the start of the file until two CRLFs have
* been seen.
* The file is left unmodified.
*/
uint32_t Sodaq_R4X::httpGetHeaderSize(const char* filename)
{
    uint32_t file_size;

    bool status = getFileSize(filename, file_size);
    if (!status) {
        return 0;
    }

    int state = 0; // 0 nothing, 1=CR, 2=CRLF, 3=CRLFCR, 4=CRLFCRLF
    uint8_t buffer[64];
    uint32_t offset = 0;

    while (offset < file_size && state != 4) {
        size_t size;
        size = readFilePartial(filename, buffer, sizeof(buffer), offset);

        if (size == 0) {
            return 0;
        }

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

size_t Sodaq_R4X::httpGetPartial(uint8_t* buffer, size_t size, uint32_t offset)
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

// Creates an HTTP POST request and optionally returns the received data.
// Note. Endpoint should include the initial "/".
// The UBlox device stores the received data in http_last_response_<profile_id>
uint32_t Sodaq_R4X::httpPost(const char* server, uint16_t port, const char* endpoint,
                             char* responseBuffer, size_t responseSize,
                             const char* sendBuffer, size_t sendSize, uint32_t timeout, bool useURC)
{
    deleteFile(HTTP_SEND_TMP_FILENAME); // cleanup the file first (if exists)

    if (!writeFile(HTTP_SEND_TMP_FILENAME, (uint8_t*)sendBuffer, sendSize)) {
        debugPrintln(DEBUG_STR_ERROR "Could not create the http tmp file!");
        return 0;
    }

    return httpPostFromFile(server, port, endpoint, responseBuffer, responseSize, HTTP_SEND_TMP_FILENAME, timeout, useURC);
}

// Creates an HTTP POST request and optionally returns the received data.
// Note. Endpoint should include the initial "/".
// The UBlox device stores the received data in http_last_response_<profile_id>
// Request body must first be prepared in a file on the modem
uint32_t Sodaq_R4X::httpPostFromFile(const char* server, uint16_t port, const char* endpoint,
                                     char* responseBuffer, size_t responseSize,
                                     const char* fileName, uint32_t timeout, bool useURC)
{
    // First just handle the request and let the file be read into the UBlox file system
    uint32_t file_size = httpRequestFromFile(server, port, endpoint, POST, responseBuffer, responseSize, fileName,
                                             timeout, useURC);
    if (file_size == 0) {
        return 0;
    }

    // Find out the header size
    _httpGetHeaderSize = httpGetHeaderSize(HTTP_RECEIVE_FILENAME);

    debugPrint("[httpPost] header size: ");
    debugPrintln(_httpGetHeaderSize);

    if (_httpGetHeaderSize == 0) {
        return 0;
    }

    if (!responseBuffer) {
        return file_size - _httpGetHeaderSize;
    }

    // Fill the buffer starting from the header
    return readFilePartial(HTTP_RECEIVE_FILENAME, (uint8_t*)responseBuffer, responseSize, _httpGetHeaderSize);
}

// Creates an HTTP request using the (optional) given buffer and
// (optionally) returns the received data.
// endpoint should include the initial "/".
size_t Sodaq_R4X::httpRequest(const char* server, uint16_t port, const char* endpoint, HttpRequestTypes requestType,
                              char* responseBuffer, size_t responseSize,
                              const char* sendBuffer, size_t sendSize, uint32_t timeout, bool useURC)
{
    // before starting the actual http request, create any files needed in the fs of the modem
    // that way there is a chance to abort sending the http req command in case of an fs error
    if (requestType == PUT || requestType == POST) {
        if (!sendBuffer || sendSize == 0) {
            debugPrintln(DEBUG_STR_ERROR "There is no sendBuffer or sendSize set!");
            return 0;
        }

        deleteFile(HTTP_SEND_TMP_FILENAME); // cleanup the file first (if exists)

        if (!writeFile(HTTP_SEND_TMP_FILENAME, (uint8_t*)sendBuffer, sendSize)) {
            debugPrintln(DEBUG_STR_ERROR "Could not create the http tmp file!");
            return 0;
        }

        return httpRequestFromFile(server, port, endpoint, requestType, responseBuffer, responseSize,
                                   HTTP_SEND_TMP_FILENAME, timeout, useURC);
    }

    // reset http profile 0
    println("AT+UHTTP=0");
    if (readResponse() != GSMResponseOK) {
        return 0;
    }

    deleteFile(HTTP_RECEIVE_FILENAME); // cleanup the file first (if exists)

    if (requestType >= HttpRequestTypesMAX) {
        debugPrintln(DEBUG_STR_ERROR "Unknown request type!");
        return 0;
    }

    // set server host name
    print("AT+UHTTP=0,");
    print(isValidIPv4(server) ? "0,\"" : "1,\"");
    print(server);
    println("\"");
    if (readResponse() != GSMResponseOK) {
        return 0;
    }

    // set port
    if (port != 80) {
        print("AT+UHTTP=0,5,");
        println(port);

        if (readResponse() != GSMResponseOK) {
            return 0;
        }
    }

    // reset the success bit before calling a new request
    _httpRequestSuccessBit[requestType] = TriBoolUndefined;

    print("AT+UHTTPC=0,");
    print(requestType < sizeof(httpRequestMapping) ? httpRequestMapping[requestType] : 1);
    print(",\"");
    print(endpoint);
    println("\",\"\""); // empty filename = default = "http_last_response_0" (DEFAULT_HTTP_RECEIVE_FILENAME)

    if (readResponse() != GSMResponseOK) {
        return 0;
    }

    // check for success while checking URCs
    // This loop relies on readResponse being called via isAlive()
    uint32_t start = millis();
    uint32_t delay_count = 50;
    while ((_httpRequestSuccessBit[requestType] == TriBoolUndefined) && !is_timedout(start, timeout)) {
        if (useURC) {
            isAlive();
            if (_httpRequestSuccessBit[requestType] != TriBoolUndefined) {
                break;
            }
        }
        else {
            uint32_t size;
            getFileSize(HTTP_RECEIVE_FILENAME, size);
            println("AT+UHTTPER=0");
            if (readResponse(NULL, 100) == GSMResponseOK) {
                break;
            }
        }

        sodaq_wdt_safe_delay(delay_count);
        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 250;
        }
    }

    if (_httpRequestSuccessBit[requestType] == TriBoolTrue) {
        uint32_t file_size;
        if (!getFileSize(HTTP_RECEIVE_FILENAME, file_size)) {
            debugPrintln(DEBUG_STR_ERROR "Could not determine file size");
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
        debugPrintln(DEBUG_STR_ERROR "An error occurred with the http request!");
        return 0;
    }
    else {
        debugPrintln(DEBUG_STR_ERROR "Timed out waiting for a response for the http request!");
        return 0;
    }

    return 0;
}

// Creates an HTTP request using the (optional) given buffer and
// (optionally) returns the received data.
// endpoint should include the initial "/".
// Request body must first be prepared in a file on the modem
size_t Sodaq_R4X::httpRequestFromFile(const char* server, uint16_t port, const char* endpoint, HttpRequestTypes requestType,
                                      char* responseBuffer, size_t responseSize,
                                      const char* fileName, uint32_t timeout, bool useURC)
{
    if (requestType != PUT && requestType != POST) {
        return 0;
    }

    // reset http profile 0
    println("AT+UHTTP=0");
    if (readResponse() != GSMResponseOK) {
        return 0;
    }

    deleteFile(HTTP_RECEIVE_FILENAME); // cleanup the file first (if exists)

    if (requestType >= HttpRequestTypesMAX) {
        debugPrintln(DEBUG_STR_ERROR "Unknown request type!");
        return 0;
    }

    // set server host name
    print("AT+UHTTP=0,");
    print(isValidIPv4(server) ? "0,\"" : "1,\"");
    print(server);
    println("\"");
    if (readResponse() != GSMResponseOK) {
        return 0;
    }

    // set port
    if (port != 80) {
        print("AT+UHTTP=0,5,");
        println(port);

        if (readResponse() != GSMResponseOK) {
            return 0;
        }
    }

    // reset the success bit before calling a new request
    _httpRequestSuccessBit[requestType] = TriBoolUndefined;

    print("AT+UHTTPC=0,");
    print(requestType < sizeof(httpRequestMapping) ? httpRequestMapping[requestType] : 1);
    print(",\"");
    print(endpoint);
    print("\",\"\",\"");
    print(fileName);
    println(requestType == POST ? "\",1" : "\"\n");

    if (readResponse() != GSMResponseOK) {
        return 0;
    }

    // check for success while checking URCs
    // This loop relies on readResponse being called via isAlive()
    uint32_t start = millis();
    uint32_t delay_count = 50;
    while ((_httpRequestSuccessBit[requestType] == TriBoolUndefined) && !is_timedout(start, timeout)) {
        if (useURC) {
            isAlive();
            if (_httpRequestSuccessBit[requestType] != TriBoolUndefined) {
                break;
            }
        }
        else {
            uint32_t size;
            getFileSize(HTTP_RECEIVE_FILENAME, size);
            println("AT+UHTTPER=0");
            if (readResponse(NULL, 100) == GSMResponseOK) {
                break;
            }
        }

        sodaq_wdt_safe_delay(delay_count);
        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 250;
        }
    }

    if (_httpRequestSuccessBit[requestType] == TriBoolTrue) {
        uint32_t file_size;
        if (!getFileSize(HTTP_RECEIVE_FILENAME, file_size)) {
            debugPrintln(DEBUG_STR_ERROR "Could not determine file size");
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
        debugPrintln(DEBUG_STR_ERROR "An error occurred with the http request!");
        return 0;
    }
    else {
        debugPrintln(DEBUG_STR_ERROR "Timed out waiting for a response for the http request!");
        return 0;
    }

    return 0;
}


//  Paremeter index has a range [0-4]
//  Parameters 'name' and 'value' can have a maximum length of 64 characters
//  Parameters 'name' and 'value' must not include the ':' character
bool Sodaq_R4X::httpSetCustomHeader(uint8_t index, const char* name, const char* value)
{
    print("AT+UHTTP=0,9,\"");
    print(index);
    print(':');

    if (name != NULL) {
        print(name);
        print(':');
        print(value);
    }

    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::httpClearCustomHeader(uint8_t index)
{
    return httpSetCustomHeader(index, NULL, NULL);
}


/******************************************************************************
* Files
*****************************************************************************/

bool Sodaq_R4X::deleteFile(const char* filename)
{
    // TODO escape filename characters
    print("AT+UDELFILE=\"");
    print(filename);
    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::getFileSize(const char* filename, uint32_t& size)
{
    size = 0;

    print("AT+ULSTFILE=2,\"");
    print(filename);
    println('"');

    char buffer[32];

    if (readResponse(buffer, sizeof(buffer), "+ULSTFILE: ") != GSMResponseOK) {
        return false;
    }

    return (sscanf(buffer, "%lu", &size) == 1);
}

size_t Sodaq_R4X::readFile(const char* filename, uint8_t* buffer, size_t size)
{
    // TODO escape filename characters { '"', ',', }

    if (!buffer || size == 0) {
        return 0;
    }

    // first, make sure the buffer is sufficient
    uint32_t filesize = 0;
    if (!getFileSize(filename, filesize)) {
        debugPrintln(DEBUG_STR_ERROR "Could not determine file size");
        return 0;
    }

    if (filesize > size) {
        debugPrintln(DEBUG_STR_ERROR "The buffer is not big enough to store the file");
        return 0;
    }

    print("AT+URDFILE=\"");
    print(filename);
    println('"');

    // override normal parsing process and explicitly read characters here
    // to be able to also read terminator characters within files
    char checkChar = 0;

    // reply identifier
    size_t len = readBytesUntil(' ', _inputBuffer, _inputBufferSize);
    if (len == 0 || strstr(_inputBuffer, "+URDFILE:") == NULL) {
        debugPrintln(DEBUG_STR_ERROR "+URDFILE literal is missing!");
        return 0;
    }

    // filename
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    // TODO check filename after removing quotes and escaping chars

    // filesize
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    filesize = 0; // reset the var before reading from reply string
    if (sscanf(_inputBuffer, "%lu", &filesize) != 1) {
        debugPrintln(DEBUG_STR_ERROR "Could not parse the file size!");
        return 0;
    }
    if (filesize == 0 || filesize > size) {
        debugPrintln(DEBUG_STR_ERROR "Size error!");
        return 0;
    }

    // opening quote character
    checkChar = timedRead();
    if (checkChar != '"') {
        debugPrintln(DEBUG_STR_ERROR "Missing starting character (quote)!");
        return 0;
    }

    // actual file buffer, written directly to the provided result buffer
    len = readBytes(buffer, filesize);
    if (len != filesize) {
        debugPrintln(DEBUG_STR_ERROR "File size error!");
        return 0;
    }

    // closing quote character
    checkChar = timedRead();
    if (checkChar != '"') {
        debugPrintln(DEBUG_STR_ERROR "Missing termination character (quote)!");
        return 0;
    }

    // read final OK response from modem and return the filesize
    return (readResponse() == GSMResponseOK ? filesize : 0);
}

size_t Sodaq_R4X::readFilePartial(const char* filename, uint8_t* buffer, size_t size, uint32_t offset)
{
    // TODO escape filename characters { '"', ',', }

    if (!buffer || size == 0) {
        return 0;
    }

    print("AT+URDBLOCK=\"");
    print(filename);
    print("\",");
    print(offset);
    print(',');
    println(size);

    // reply identifier
    //   +URDBLOCK: http_last_response_0,86,"..."
    // where 86 is an example of the size
    size_t len = readBytesUntil(' ', _inputBuffer, _inputBufferSize);
    if (len == 0 || strstr(_inputBuffer, "+URDBLOCK:") == NULL) {
        debugPrintln(DEBUG_STR_ERROR "+URDBLOCK literal is missing!");
        return 0;
    }

    // skip filename
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    // TODO check filename. Note, there are no quotes

    // read the number of bytes
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    uint32_t blocksize = 0; // reset the var before reading from reply string
    if (sscanf(_inputBuffer, "%lu", &blocksize) != 1) {
        debugPrintln(DEBUG_STR_ERROR "Could not parse the block size!");
        return 0;
    }
    if (blocksize == 0 || blocksize > size) {
        debugPrintln(DEBUG_STR_ERROR "Size error!");
        return 0;
    }

    // opening quote character
    char quote = timedRead();
    if (quote != '"') {
        debugPrintln(DEBUG_STR_ERROR "Missing starting character (quote)!");
        return 0;
    }

    // actual file buffer, written directly to the provided result buffer
    len = readBytes(buffer, blocksize);
    if (len != blocksize) {
        debugPrintln(DEBUG_STR_ERROR "File size error!");
        return 0;
    }

    // closing quote character
    quote = timedRead();
    if (quote != '"') {
        debugPrintln(DEBUG_STR_ERROR "Missing termination character (quote)!");
        return 0;
    }

    // read final OK response from modem and return the filesize
    return (readResponse() == GSMResponseOK ? blocksize : 0);
}

// If the file already exists, the data will be appended to the file already stored in the file system.
bool Sodaq_R4X::writeFile(const char* filename, const uint8_t* buffer, size_t size)
{
    // TODO escape filename characters
    print("AT+UDWNFILE=\"");
    print(filename);
    print("\",");
    println(size);

    if (readResponse() != GSMResponsePrompt) {
        return false;
    }

    for (size_t i = 0; i < size; i++) {
        writeByte(buffer[i]);
    }

    return (readResponse() == GSMResponseOK);
}


/******************************************************************************
* Private
*****************************************************************************/

int8_t Sodaq_R4X::checkApn(const char* requiredAPN)
{
    println("AT+CGDCONT?");

    char buffer[256];

    if (readResponse(buffer, sizeof(buffer), "+CGDCONT: ") != GSMResponseOK) {
        return -1;
    }

    if (strncmp(buffer, "1,\"IP\"", 6) == 0 && strncmp(buffer + 6, ",\"\"", 3) != 0) {
        char apn[64];
        char ip[32];

        if (sscanf(buffer + 6, ",\"%[^\"]\",\"%[^\"]\",0,0,0,0", apn, ip) != 2) { return -1; }

        if (strcmp(apn, requiredAPN) == 0) {
            if (strlen(ip) >= 7 && strcmp(ip, "0.0.0.0") != 0) { 
                return 1; 
            }
            else {
                return 0;
            }
        }
    }

    return setApn(requiredAPN) ? 0 : -1;
}

bool Sodaq_R4X::checkBandMasks(const char* bandMaskLTE, const char* bandMaskNB)
{
    // if no changes required
    if ((bandMaskLTE == BAND_MASK_UNCHANGED) && (bandMaskNB == BAND_MASK_UNCHANGED)) {
        return true;
    }

    println("AT+UBANDMASK?");

    char buffer[128];

    if (readResponse(buffer, sizeof(buffer), "+UBANDMASK: ") != GSMResponseOK) {
        return false;
    }

    char bm0[32];
    char bm1[32];
    if (sscanf(buffer, "0,%[^,],1,%s", bm0, bm1) != 2) {
        return false;
    }

    bool setLTEMask = (strcmp(bm0, bandMaskLTE) != 0) && (bandMaskLTE != BAND_MASK_UNCHANGED);
    bool setNBMask = (strcmp(bm1, bandMaskNB) != 0) && (bandMaskNB != BAND_MASK_UNCHANGED);

    // masks are both already match those requested
    if (!setLTEMask && !setNBMask) {
        return true;
    }

    print("AT+UBANDMASK=");
    if (setLTEMask) {
        print("0,");
        print(bandMaskLTE);
        if (setNBMask) {
            print(",");
        }
    }
    if (setNBMask) {
        print("1,");
        print(bandMaskNB);
    }
    println();

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    reboot();

    return true;
}

bool Sodaq_R4X::checkCFUN()
{
    println("AT+CFUN?");

    char buffer[64];

    if (readResponse(buffer, sizeof(buffer), "+CFUN: ") != GSMResponseOK) {
        return false;
    }

    return ((strcmp(buffer, "1") == 0) || setRadioActive(true));
}

bool Sodaq_R4X::checkCOPS(const char* requiredOperator, const char* requiredURAT)
{
    // If auto operator and not NB1, always send the command
    if ((strcmp(requiredOperator, AUTOMATIC_OPERATOR) == 0) && (strcmp(requiredURAT, SODAQ_R4X_NBIOT_URAT) != 0)){
        return execCommand("AT+COPS=0,2", COPS_TIMEOUT);
    }

    println("AT+COPS=3,2");

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    println("AT+COPS?");

    char buffer[64];

    if (readResponse(buffer, sizeof(buffer), "+COPS: ", COPS_TIMEOUT) != GSMResponseOK) {
        return false;
    }

    if (strcmp(requiredOperator, AUTOMATIC_OPERATOR) == 0) {
        return ((strncmp(buffer, "0", 1) == 0) || execCommand("AT+COPS=0,2", COPS_TIMEOUT));
    }
    else if ((strncmp(buffer, "1", 1) == 0) && (strncmp(buffer + 5, requiredOperator, strlen(requiredOperator)) == 0)) {
        return true;
    }
    else {
        print("AT+COPS=1,2,\"");
        print(requiredOperator);
        println('"');

        return (readResponse(NULL, 0, NULL, COPS_TIMEOUT) == GSMResponseOK);
    }
}

bool Sodaq_R4X::checkProfile(const uint8_t requiredProfile)
{
    println("AT+UMNOPROF?");

    char buffer[64];
    if (readResponse(buffer, sizeof(buffer), "+UMNOPROF: ") != GSMResponseOK) {
        return false;
    }

    if (atoi(buffer) == requiredProfile) {
        return true;
    }

    println("AT+COPS=2");
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    print("AT+UMNOPROF=");
    println(requiredProfile);
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    reboot();

    return true;
}

bool Sodaq_R4X::checkUrat(const char* requiredURAT)
{
    // Only try and skip if single URAT
    if (strlen(requiredURAT) == 1) {
        println("AT+URAT?");

        char buffer[64];

        if (readResponse(buffer, sizeof(buffer), "+URAT: ") != GSMResponseOK || strlen(buffer) == 0) {
            return false;
        }

        if (strcmp(buffer, requiredURAT) == 0) {
            return true;
        }
    }

    println("AT+COPS=2");
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    print("AT+URAT=");
    println(requiredURAT);
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    reboot();

    return true;
}

bool Sodaq_R4X::checkURC(char* buffer)
{
    if (buffer[0] != '+') {
        return false;
    }

    int param1, param2;
    char param3[1024];

    if (sscanf(buffer, "+UFOTAS: %d,%d", &param1, &param2) == 2) {
        #ifdef DEBUG
        debugPrint("Unsolicited: FOTA: ");
        debugPrint(param1);
        debugPrint(", ");
        debugPrintln(param2);
        #endif

        return true;
    }

    if (sscanf(buffer, "+UHTTPER: 0,%*d,%d", &param1) == 1) {
        debugPrint("Unsolicited: UHTTPER: ");
        debugPrintln(param1);

        _httpRequestSuccessBit[1] = param1 == 0 ? TriBoolTrue : TriBoolFalse;

        return true;
    }

    if (sscanf(buffer, "+UUHTTPCR: 0,%d,%d", &param1, &param2) == 2) {
        static uint8_t mapping[] = {
            HEAD,   // 0
            GET,    // 1
            DELETE, // 2
            PUT,    // 3
            POST,   // 4
        };

        int requestType = param1 < (int)sizeof(mapping) ? mapping[param1] : -1;
        if (requestType >= 0) {
            debugPrint("Unsolicited: UUHTTPCR: ");
            debugPrint(requestType);
            debugPrint(": ");
            debugPrintln(param2);

            if (param2 == 0) {
                _httpRequestSuccessBit[requestType] = TriBoolFalse;
            }
            else if (param2 == 1) {
                _httpRequestSuccessBit[requestType] = TriBoolTrue;
            }
        }

        return true;
    }

    if (sscanf(buffer, "+UUMQTTC: 1,%d", &param1) == 1) {
        debugPrint("Unsolicited: MQTT login result: ");
        debugPrintln(param1);

        _mqttLoginResult = param1;

        return true;
    }

    if (sscanf(buffer, "+UUMQTTC: 4,%d,%d,\"%[^\"]\"", &param1, &param2, param3) == 3) {
        debugPrint("Unsolicited: MQTT subscription result: ");
        debugPrint(param1);
        debugPrint(", ");
        debugPrint(param2);
        debugPrint(", ");
        debugPrintln(param3);

       _mqttSubscribeReason = param1;

        return true;
    }

    if (sscanf(buffer, "+UUMQTTCM: 6,%d", &param1) == 1) {
        debugPrint("Unsolicited: MQTT pending messages:");
        debugPrintln(param1);

        _mqttPendingMessages = param1;

        return true;
    }

    if (sscanf(buffer, "+UUSORD: %d,%d", &param1, &param2) == 2) {
        debugPrint("Unsolicited: Socket ");
        debugPrint(param1);
        debugPrint(": ");
        debugPrintln(param2);

        if (param1 >= 0 && param1 < SOCKET_COUNT) {
            _socketPendingBytes[param1] = param2;
        }

        return true;
    }

    if (sscanf(buffer, "+UUSORF: %d,%d", &param1, &param2) == 2) {
        debugPrint("Unsolicited: Socket ");
        debugPrint(param1);
        debugPrint(": ");
        debugPrintln(param2);

        if (param1 >= 0 && param1 < SOCKET_COUNT) {
            _socketPendingBytes[param1] = param2;
        }

        return true;
    }

    if (sscanf(buffer, "+UUSOCL: %d", &param1) == 1) {
        debugPrint("Unsolicited: Socket ");
        debugPrintln(param1);

        if (param1 >= 0 && param1 < SOCKET_COUNT) {
            _socketClosedBit[param1] = true;
        }

        return true;
    }

    return false;
}

bool Sodaq_R4X::doSIMcheck()
{
    const uint8_t retry_count = 10;

    for (uint8_t i = 0; i < retry_count; i++) {
        if (i > 0) {
            sodaq_wdt_safe_delay(250);
        }

        SimStatuses simStatus = getSimStatus();
        if (simStatus == SimNeedsPin) {
            if (_pin == 0 || *_pin == '\0' || !setSimPin(_pin)) {
                debugPrintln(DEBUG_STR_ERROR "SIM needs a PIN but none was provided, or setting it failed!");
                return false;
            }
        }
        else if (simStatus == SimReady) {
            return true;
        }
    }

    return false;
}

bool Sodaq_R4X::setNetworkLEDState()
{
    print("AT+UGPIOC=");
    print(NETWORK_STATUS_GPIO_ID);
    print(',');
    (_networkStatusLED) ? println("2") : println("255");

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::isValidIPv4(const char* str)
{
    uint8_t  segs  = 0; // Segment count
    uint8_t  chcnt = 0; // Character count within segment
    uint16_t accum = 0; // Accumulator for segment

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

/**
 * 1. check echo
 * 2. check ok
 * 3. check error
 * 4. if response prefix is not empty, check response prefix, append if multiline
 * 5. check URC, if handled => continue
 * 6. if response prefis is empty, return the whole line return line buffer, append if multiline
*/
GSMResponseTypes Sodaq_R4X::readResponse(char* outBuffer, size_t outMaxSize, const char* prefix, uint32_t timeout)
{
    bool usePrefix    = prefix != NULL && prefix[0] != 0;
    bool useOutBuffer = outBuffer != NULL && outMaxSize > 0;

    uint32_t from = NOW;

    size_t outSize = 0;

    if (outBuffer) {
        outBuffer[0] = 0;
    }

    while (!is_timedout(from, timeout)) {
        int count = readLn(_inputBuffer, _inputBufferSize, 250); // 250ms, how many bytes at which baudrate?
        sodaq_wdt_reset();

        if (count <= 0) {
            continue;
        }

        debugPrint("<< ");
        debugPrintln(_inputBuffer);

        if (startsWith(STR_AT, _inputBuffer)) {
            continue; // skip echoed back command
        }

        if (startsWith(STR_RESPONSE_OK, _inputBuffer)) {
            return GSMResponseOK;
        }

        if (startsWith(STR_RESPONSE_ERROR, _inputBuffer) ||
                startsWith(STR_RESPONSE_CME_ERROR, _inputBuffer) ||
                startsWith(STR_RESPONSE_CMS_ERROR, _inputBuffer)) {
            return GSMResponseError;
        }

        if ((_inputBuffer[0] == STR_RESPONSE_SOCKET_PROMPT) || (_inputBuffer[0] == STR_RESPONSE_FILE_PROMPT)) {
            return GSMResponsePrompt;
        }

        bool hasPrefix = usePrefix && useOutBuffer && startsWith(prefix, _inputBuffer);

        if (!hasPrefix && checkURC(_inputBuffer)) {
            continue;
        }

        if (hasPrefix || (!usePrefix && useOutBuffer)) {
            if (outSize > 0 && outSize < outMaxSize - 1) {
                outBuffer[outSize++] = LF;
            }

            if (outSize < outMaxSize - 1) {
                char* inBuffer = _inputBuffer;
                if (hasPrefix) {
                    int i = strlen(prefix);
                    count -= i;
                    inBuffer += i;
                }
                if (outSize + count > outMaxSize - 1) {
                    count = outMaxSize - 1 - outSize;
                }
                memcpy(outBuffer + outSize, inBuffer, count);
                outSize += count;
                outBuffer[outSize] = 0;
            }
        }
    }

    debugPrintln("<< timed out");

    return GSMResponseTimeout;
}

void Sodaq_R4X::reboot()
{
    println("AT+CFUN=15");

    // wait up to 2000ms for the modem to come up
    uint32_t start = millis();

    while ((readResponse() != GSMResponseOK) && !is_timedout(start, 2000)) {}

    // wait for the reboot to start
    sodaq_wdt_safe_delay(REBOOT_DELAY);

    while (!is_timedout(start, REBOOT_TIMEOUT)) {
        if (getSimStatus() == SimReady) {
            break;
        }
    }

    // echo off again after reboot
    execCommand("ATE0");

    // extra read just to clear the input stream
    readResponse(NULL, 0, NULL, 250);
}

bool Sodaq_R4X::setSimPin(const char* simPin)
{
    print("AT+CPIN=\"");
    print(simPin);
    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::waitForSignalQuality(uint32_t timeout)
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


/******************************************************************************
* Utils
*****************************************************************************/

uint32_t Sodaq_R4X::convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec)
{
    struct tm tm;

    tm.tm_isdst = -1;
    tm.tm_yday  = 0;
    tm.tm_wday  = 0;
    tm.tm_year  = y + EPOCH_TIME_YEAR_OFF;
    tm.tm_mon   = m - 1;
    tm.tm_mday  = d;
    tm.tm_hour  = h;
    tm.tm_min   = min;
    tm.tm_sec   = sec;

    return mktime(&tm);
}

bool Sodaq_R4X::startsWith(const char* pre, const char* str)
{
    return (strncmp(pre, str, strlen(pre)) == 0);
}


/******************************************************************************
* Generic
*****************************************************************************/

// Returns true if the modem is on.
bool Sodaq_R4X::isOn() const
{
    if (_onoff) {
        return _onoff->isOn();
    }

    // No onoff. Let's assume it is on.
    return true;
}

void Sodaq_R4X::writeProlog()
{
    if (!_appendCommand) {
        debugPrint(">> ");
        _appendCommand = true;
    }
}

// Write a byte, as binary data
size_t Sodaq_R4X::writeByte(uint8_t value)
{
    return _modemStream->write(value);
}

size_t Sodaq_R4X::print(const String& buffer)
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_R4X::print(const char buffer[])
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_R4X::print(char value)
{
    writeProlog();
    debugPrint(value);

    return _modemStream->print(value);
};

size_t Sodaq_R4X::print(unsigned char value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_R4X::print(int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_R4X::print(unsigned int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_R4X::print(long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_R4X::print(unsigned long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_R4X::println(const __FlashStringHelper *ifsh)
{
    return print(ifsh) + println();
}

size_t Sodaq_R4X::println(const String &s)
{
    return print(s) + println();
}

size_t Sodaq_R4X::println(const char c[])
{
    return print(c) + println();
}

size_t Sodaq_R4X::println(char c)
{
    return print(c) + println();
}

size_t Sodaq_R4X::println(unsigned char b, int base)
{
    return print(b, base) + println();
}

size_t Sodaq_R4X::println(int num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_R4X::println(unsigned int num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_R4X::println(long num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_R4X::println(unsigned long num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_R4X::println(double num, int digits)
{
    writeProlog();
    debugPrint(num, digits);

    return _modemStream->println(num, digits);
}

size_t Sodaq_R4X::println(const Printable& x)
{
    return print(x) + println();
}

size_t Sodaq_R4X::println()
{
    debugPrintln();
    size_t i = print(CR);
    _appendCommand = false;
    return i;
}

// Initializes the input buffer and makes sure it is only initialized once.
// Safe to call multiple times.
void Sodaq_R4X::initBuffer()
{
    debugPrintln("[initBuffer]");

    // make sure the buffers are only initialized once
    if (!_isBufferInitialized) {
        _inputBuffer = static_cast<char*>(malloc(_inputBufferSize));
        _isBufferInitialized = true;
    }
}

// Sets the modem stream.
void Sodaq_R4X::setModemStream(Stream& stream)
{
    _modemStream = &stream;
}

// Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
int Sodaq_R4X::timedRead(uint32_t timeout) const
{
    uint32_t _startMillis = millis();

    do {
        int c = _modemStream->read();

        if (c >= 0) {
            return c;
        }
    } while (millis() - _startMillis < timeout);

    return -1; // -1 indicates timeout
}

// Fills the given "buffer" with characters read from the modem stream up to "length"
// maximum characters and until the "terminator" character is found or a character read
// times out (whichever happens first).
// The buffer does not contain the "terminator" character or a null terminator explicitly.
// Returns the number of characters written to the buffer, not including null terminator.
size_t Sodaq_R4X::readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout)
{
    if (length < 1) {
        return 0;
    }

    size_t index = 0;

    while (index < length) {
        int c = timedRead(timeout);

        if (c < 0 || c == terminator) {
            break;
        }

        *buffer++ = static_cast<char>(c);
        index++;
    }
    if (index < length) {
        *buffer = '\0';
    }

    return index; // return number of characters, not including null terminator
}

// Fills the given "buffer" with up to "length" characters read from the modem stream.
// It stops when a character read times out or "length" characters have been read.
// Returns the number of characters written to the buffer.
size_t Sodaq_R4X::readBytes(uint8_t* buffer, size_t length, uint32_t timeout)
{
    size_t count = 0;

    while (count < length) {
        int c = timedRead(timeout);

        if (c < 0) {
            break;
        }

        *buffer++ = static_cast<uint8_t>(c);
        count++;
    }

    return count;
}

// Reads a line (up to the SODAQ_GSM_TERMINATOR) from the modem stream into the "buffer".
// The buffer is terminated with null.
// Returns the number of bytes read, not including the null terminator.
size_t Sodaq_R4X::readLn(char* buffer, size_t size, uint32_t timeout)
{
    // Use size-1 to leave room for a string terminator
    size_t len = readBytesUntil(SODAQ_GSM_TERMINATOR[SODAQ_GSM_TERMINATOR_LEN - 1], buffer, size - 1, timeout);

    // check if the terminator is more than 1 characters, then check if the first character of it exists
    // in the calculated position and terminate the string there
    if ((SODAQ_GSM_TERMINATOR_LEN > 1) && (buffer[len - (SODAQ_GSM_TERMINATOR_LEN - 1)] == SODAQ_GSM_TERMINATOR[0])) {
        len -= SODAQ_GSM_TERMINATOR_LEN - 1;
    }

    // terminate string, there should always be room for it (see size-1 above)
    buffer[len] = '\0';

    return len;
}


/******************************************************************************
 * OnOff
 *****************************************************************************/

Sodaq_SARA_R4XX_OnOff::Sodaq_SARA_R4XX_OnOff()
{
    #ifdef PIN_SARA_ENABLE
    // First write the output value, and only then set the output mode.
    digitalWrite(SARA_ENABLE, LOW);
    pinMode(SARA_ENABLE, OUTPUT);

    digitalWrite(SARA_TX_ENABLE, LOW);
    pinMode(SARA_TX_ENABLE, OUTPUT);
    #endif

    _onoff_status = false;
}

void Sodaq_SARA_R4XX_OnOff::on()
{
    #ifdef PIN_SARA_ENABLE
    digitalWrite(SARA_ENABLE, HIGH);
    digitalWrite(SARA_TX_ENABLE, HIGH);

    pinMode(SARA_R4XX_TOGGLE, OUTPUT);
    digitalWrite(SARA_R4XX_TOGGLE, LOW);
    // We should be able to reduce this to 50ms or something
    sodaq_wdt_safe_delay(2000);
    pinMode(SARA_R4XX_TOGGLE, INPUT);

    _onoff_status = true;
    #endif
}

void Sodaq_SARA_R4XX_OnOff::off()
{
    #ifdef PIN_SARA_ENABLE
    digitalWrite(SARA_ENABLE, LOW);
    digitalWrite(SARA_TX_ENABLE, LOW);

    _onoff_status = false;
    #endif
}

bool Sodaq_SARA_R4XX_OnOff::isOn()
{
    return _onoff_status;
}
