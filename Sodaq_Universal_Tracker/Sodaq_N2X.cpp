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

#include "Sodaq_N2X.h"
#include "Sodaq_wdt.h"
#include "time.h"

#define DEBUG

#define EPOCH_TIME_OFF          946684800  /* This is 1st January 2000, 00:00:00 in epoch time */
#define EPOCH_TIME_YEAR_OFF     100        /* years since 1900 */
#define COPS_TIMEOUT            180000
#define ISCONNECTED_CSQ_TIMEOUT 10000

#define SODAQ_GSM_TERMINATOR "\r\n"
#define SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE 1024
#define SODAQ_GSM_TERMINATOR_LEN (sizeof(SODAQ_GSM_TERMINATOR) - 1)

#define STR_AT                 "AT"
#define STR_RESPONSE_OK        "OK"
#define STR_RESPONSE_ERROR     "ERROR"
#define STR_RESPONSE_CME_ERROR "+CME ERROR:"
#define STR_RESPONSE_CMS_ERROR "+CMS ERROR:"

#define NIBBLE_TO_HEX_CHAR(i)  ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i)         ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i)          (i & 0x0F)
#define HEX_CHAR_TO_NIBBLE(c)  ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

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

typedef struct NameValuePair {
    const char* Name;
    bool Value;
} NameValuePair;

const uint8_t nConfigCount = 6;
static NameValuePair nConfig[nConfigCount] = {
    { "AUTOCONNECT",             false },
    { "CR_0354_0338_SCRAMBLING", true  },
    { "CR_0859_SI_AVOID",        false },
    { "COMBINE_ATTACH",          false },
    { "CELL_RESELECTION",        false },
    { "ENABLE_BIP",              false }
};

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms) { return (millis() - from) > nr_ms; }


/******************************************************************************
* Main
*****************************************************************************/

Sodaq_N2X::Sodaq_N2X() :
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

    memset(_socketPendingBytes, 0, sizeof(_socketPendingBytes));
}

// Initializes the modem instance. Sets the modem stream and the on-off power pins.
void Sodaq_N2X::init(Sodaq_OnOffBee* onoff, Stream& stream, uint8_t cid)
{
    debugPrintln("[init] started.");

    initBuffer(); // safe to call multiple times

    setModemStream(stream);

    _onoff = onoff;
    _cid   = cid;
}

// Turns the modem on and returns true if successful.
bool Sodaq_N2X::on()
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

    return isOn(); // this essentially means isOn() && isAlive()
}

// Turns the modem off and returns true if successful.
bool Sodaq_N2X::off()
{
    // No matter if it is on or off, turn it off.
    if (_onoff) {
        _onoff->off();
    }

    return !isOn();
}

// Turns on and initializes the modem, then connects to the network and activates the data connection.
bool Sodaq_N2X::connect(const char* apn, const char* cdp, const char* forceOperator, const char* band)
{
    if (!on()) {
        return false;
    }

    purgeAllResponsesRead();

    if (!setVerboseErrors(true) || !setBand(band) || !checkAndApplyNconfig()) {
        return false;
    }

    reboot();

    if (!on()) {
        return false;
    }

    purgeAllResponsesRead();

    return setVerboseErrors(true)      &&
           setRadioActive(true)        &&
           setIndicationsActive(false) &&
           setApn(apn)                 &&
           setCdp(cdp)                 &&
           setOperator(forceOperator)  &&
           waitForSignalQuality()      &&
           attachGprs();
}

// Disconnects the modem from the network.
bool Sodaq_N2X::disconnect()
{
    return execCommand("AT+CGATT=0", 40000);
}


/******************************************************************************
* Public
*****************************************************************************/

bool Sodaq_N2X::attachGprs(uint32_t timeout)
{
    uint32_t start = millis();
    uint32_t delay_count = 500;

    while (!is_timedout(start, timeout)) {
        if (isAttached()) {
            return true;
        }

        sodaq_wdt_safe_delay(delay_count);

        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 1000;
        }
    }

    return false;
}

// Gets Integrated Circuit Card ID.
// Should be provided with a buffer of at least 21 bytes.
// Returns true if successful.
bool Sodaq_N2X::getCCID(char* buffer, size_t size)
{
    if (buffer == NULL || size < 20 + 1) {
        return false;
    }

    println("AT+CCID");

    return (readResponse(buffer, size, "+CCID: ") == GSMResponseOK) && (strlen(buffer) > 0);
}

bool Sodaq_N2X::getOperatorInfo(uint16_t* mcc, uint16_t* mnc)
{
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

bool Sodaq_N2X::getOperatorInfoString(char* buffer, size_t size)
{
    if (size < 32 + 1) {
         return false;
    }

    buffer[0] = 0;

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

bool Sodaq_N2X::getCellId(uint16_t* tac, uint32_t* cid)
{
    println("AT+CEREG=2");

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    println("AT+CEREG?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+CEREG: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {

        if (sscanf(responseBuffer, "2,%*d,\"%hx\",\"%x\",", tac, cid) == 2) {
            return true;
        }
    }

    return false;
}

bool Sodaq_N2X::getEpoch(uint32_t* epoch)
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

bool Sodaq_N2X::getFirmwareVersion(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("AT+CGMR", DEFAULT_READ_MS, buffer, size);
}

bool Sodaq_N2X::getFirmwareRevision(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("ATI9", DEFAULT_READ_MS, buffer, size);
}

// Gets International Mobile Equipment Identity.
// Should be provided with a buffer of at least 16 bytes.
// Returns true if successful.
bool Sodaq_N2X::getIMEI(char* buffer, size_t size)
{
    if (buffer == NULL || size < 15 + 1) {
        return false;
    }

    println("AT+CGSN=1");

    return (readResponse(buffer, size, "+CGSN: ") == GSMResponseOK) && (strlen(buffer) > 0);
}

bool Sodaq_N2X::execCommand(const char* command, uint32_t timeout, char* buffer, size_t size)
{
    println(command);

    return (readResponse(buffer, size, NULL, timeout) == GSMResponseOK);
}

// Returns true if the modem replies to "AT" commands without timing out.
bool Sodaq_N2X::isAlive()
{
    return execCommand(STR_AT, 450);
}

// Returns true if the modem is attached to the network and has an activated data connection.
bool Sodaq_N2X::isAttached()
{
    println("AT+CGATT?");

    char buffer[16];

    if (readResponse(buffer, sizeof(buffer), "+CGATT: ", 10 * 1000) != GSMResponseOK) {
        return false;
    }

    return (strcmp(buffer, "1") == 0);
}

// Returns true if the modem is connected to the network and IP address is not 0.0.0.0.
bool Sodaq_N2X::isConnected()
{
    return isAttached() && waitForSignalQuality(ISCONNECTED_CSQ_TIMEOUT);
}

bool Sodaq_N2X::overrideNconfigParam(const char* param, bool value)
{
    for (uint8_t i = 0; i < nConfigCount; i++) {
        if (strcmp(nConfig[i].Name, param) == 0) {
            nConfig[i].Value = value;
            return true;
        }
    }

    return false;
}

bool Sodaq_N2X::ping(const char* ip)
{
    print("AT+NPING=\"");
    print(ip);
    println('"');

    return (readResponse() == GSMResponseOK);
}

void Sodaq_N2X::purgeAllResponsesRead()
{
    uint32_t start = millis();

    // make sure all the responses within the timeout have been read
    while ((readResponse(NULL, 0, NULL, 1000) != GSMResponseTimeout) && !is_timedout(start, 2000)) {}
}

bool Sodaq_N2X::setApn(const char* apn)
{
    print("AT+CGDCONT=");
    print(_cid);
    print(",\"IP\",\"");
    print(apn);
    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::setBand(uint8_t band)
{
    print("AT+NBAND=");
    println(band);

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::setBand(const char* band)
{
    print("AT+NBAND=");
    println(band);

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::setCdp(const char* cdp)
{
    if (cdp == NULL || cdp[0] == 0) {
        debugPrintln("Skipping empty CDP");
        return true;
    }

    print("AT+NCDP=\"");
    print(cdp);
    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::setIndicationsActive(bool on)
{
    print("AT+NSMI=");
    println(on ? '1' : '0');

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    print("AT+NNMI=");
    println(on ? '1' : '0');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::setOperator(const char* opr)
{
    if (opr == NULL || opr[0] == 0) {
        debugPrintln("Skipping empty operator");
        return true;
    }

    if (strcmp(opr, "0") == 0) {
        println("AT+COPS=0");
    }
    else {
        print("AT+COPS=1,2,\"");
        print(opr);
        println('"');
    }

    return (readResponse(NULL, 0, NULL, COPS_TIMEOUT) == GSMResponseOK);
}

bool Sodaq_N2X::setRadioActive(bool on)
{
    print("AT+CFUN=");
    println(on ? '1' : '0');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::setVerboseErrors(bool on)
{
    print("AT+CMEE=");
    println(on ? '1' : '0');

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
int8_t Sodaq_N2X::convertCSQ2RSSI(uint8_t csq) const
{
    return -113 + 2 * csq;
}

uint8_t Sodaq_N2X::convertRSSI2CSQ(int8_t rssi) const
{
    return (rssi + 113) / 2;
}

// Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
// Returns true if successful.
bool Sodaq_N2X::getRSSIAndBER(int8_t* rssi, uint8_t* ber)
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

bool Sodaq_N2X::socketClose(uint8_t socketID, bool async)
{
    // This parameter only applies to TCP sockets
    // The N2 series only supports UDP
    UNUSED(async);

    // only Datagram/UDP is supported
    print("AT+NSOCL=");
    println(socketID);

    if (readResponse() != GSMResponseOK) {
        return false;
    }

    _socketPendingBytes[socketID] = 0;

    return true;
}

int Sodaq_N2X::socketCreate(uint16_t localPort, Protocols protocol)
{
    // The N2 series only supports UDP
    UNUSED(protocol);

    // only Datagram/UDP is supported
    print("AT+NSOCR=\"DGRAM\",17,");
    print(localPort);
    println(",1");

    char buffer[32];

    if (readResponse(buffer, sizeof(buffer)) != GSMResponseOK) {
        return SOCKET_FAIL;
    }

    int socketID;

    if ((sscanf(buffer, "%d", &socketID) != 1) || (socketID < 0) || (socketID > SOCKET_COUNT)) {
        return SOCKET_FAIL;
    }

    _socketPendingBytes[socketID] = 0;

    return socketID;
}

size_t Sodaq_N2X::socketGetPendingBytes(uint8_t socketID)
{
    return _socketPendingBytes[socketID];
}

bool Sodaq_N2X::socketHasPendingBytes(uint8_t socketID)
{
    return _socketPendingBytes[socketID] > 0;
}

size_t Sodaq_N2X::socketReceive(uint8_t socketID, uint8_t* buffer, size_t size)
{
    if (!socketHasPendingBytes(socketID)) {
        // no URC has happened, no socket to read
        debugPrintln("Reading from without available bytes!");
        return 0;
    }

    size = min(size, min(SODAQ_N2X_MAX_UDP_BUFFER, _socketPendingBytes[socketID]));

    char   outBuffer[SODAQ_N2X_MAX_UDP_BUFFER];
    int    retSocketID;
    size_t retSize;

    print("AT+NSORF=");
    print(socketID);
    print(',');
    println(size);

    if (readResponse(outBuffer, sizeof(outBuffer)) != GSMResponseOK) {
        return 0;
    }

    if (sscanf(outBuffer, "%d,\"%*[^\"]\",%*d,%d,\"%[^\"]\",%*d", &retSocketID, &retSize, outBuffer) != 3) {
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

size_t Sodaq_N2X::socketSend(uint8_t socketID, const char* remoteIP, const uint16_t remotePort, const uint8_t* buffer, size_t size)
{
    if (size > SODAQ_MAX_UDP_SEND_MESSAGE_SIZE) {
        debugPrintln("Message exceeded maximum size!");
        return 0;
    }

    // only Datagram/UDP is supported
    print("AT+NSOST=");
    print(socketID);
    print(",\"");
    print(remoteIP);
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

    if (readResponse(outBuffer, sizeof(outBuffer)) != GSMResponseOK) {
        return 0;
    }

    int retSocketID;
    int sentLength;

    if ((sscanf(outBuffer, "%d,%d", &retSocketID, &sentLength) != 2) || (retSocketID < 0) || (retSocketID > SOCKET_COUNT)) {
        return 0;
    }

    return sentLength;
}

bool Sodaq_N2X::socketWaitForReceive(uint8_t socketID, uint32_t timeout)
{
    if (socketHasPendingBytes(socketID)) {
        return true;
    }

    uint32_t startTime = millis();

    while (!socketHasPendingBytes(socketID) && (millis() - startTime) < timeout) {
        isAlive();
        sodaq_wdt_safe_delay(10);
    }

    return socketHasPendingBytes(socketID);
}


/******************************************************************************
* Messsages
*****************************************************************************/

bool Sodaq_N2X::getReceivedMessagesCount(ReceivedMessageStatus* status)
{
    char buffer[128];

    if (!execCommand("AT+NQMGS", DEFAULT_READ_MS, buffer, sizeof(buffer))) {
        return false;
    }

    int buffered;
    int received;
    int dropped;

    if (sscanf(buffer, "BUFFERED=%d,RECEIVED=%d,DROPPED=%d", &buffered, &received, &dropped) != 3) {
        return false;
    }

    status->pending           = buffered;
    status->receivedSinceBoot = received;
    status->droppedSinceBoot  = dropped;

    return true;
}

int Sodaq_N2X::getSentMessagesCount(SentMessageStatus filter)
{
    char buffer[128];

    if (!execCommand("AT+NQMGS", DEFAULT_READ_MS, buffer, sizeof(buffer))) {
        return -1;
    }

    int pendingCount;
    int errorCount;

    if (sscanf(buffer, "PENDING=%d,SENT=%*d,ERROR=%d", &pendingCount, &errorCount) == 2) {
        if (filter == Pending) {
            return pendingCount;
        }

        if (filter == Error) {
            return errorCount;
        }
    }

    return -1;
}

// NOTE! Need to send data ( sendMessage() ) before receiving
size_t Sodaq_N2X::receiveMessage(char* buffer, size_t size)
{
    char outBuffer[1024];

    if (!execCommand("AT+NMGR", DEFAULT_READ_MS, outBuffer, sizeof(outBuffer))) {
        return 0;
    }

    size_t receivedLength;

    if (sscanf(outBuffer, "%d,\"%s\"", &receivedLength, buffer) == 2) {
        // length contains the length of the passed buffer
        // this guards against overflowing the passed buffer
        if (receivedLength * 2 <= size) {
            return receivedLength * 2;
        }
    }

    return 0;
}

bool Sodaq_N2X::sendMessage(const uint8_t* buffer, size_t size)
{
    if (size > 512) {
        return false;
    }

    print("AT+NMGS=");
    print(size);
    print(",\"");

    for (uint16_t i = 0; i < size; ++i) {
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(buffer[i]))));
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(buffer[i]))));
    }

    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::sendMessage(const char* str)
{
    return sendMessage((const uint8_t*)str, strlen(str));
}

bool Sodaq_N2X::sendMessage(String str)
{
    return sendMessage(str.c_str());
}


/******************************************************************************
* Private
*****************************************************************************/

bool Sodaq_N2X::checkAndApplyNconfig()
{
    println("AT+NCONFIG?");

    char buffer[1024];

    if (readResponse(buffer, sizeof(buffer), "+NCONFIG: ") != GSMResponseOK) {
        return false;
    }

    size_t size = strlen(buffer);
    size_t offs = 0;

    while (offs < size && buffer[offs] != 0) {
        char name[32];
        char value[32];

        if (sscanf(buffer + offs, "\"%[^\"]\",\"%[^\"]\"", name, value) != 2) {
            break;
        }

        for (uint8_t i = 0; i < nConfigCount; i++) {
            if (strcmp(nConfig[i].Name, name) == 0) {
                debugPrint(nConfig[i].Name);

                const char* v = nConfig[i].Value ? "TRUE" : "FALSE";

                if (strcmp(value, v) == 0) {
                    debugPrintln("... OK");
                }
                else {
                    debugPrintln("... CHANGE");
                    setNconfigParam(nConfig[i].Name, v);
                }

                break;
            }
        }

        offs += 6;
        while (offs < sizeof(buffer) && buffer[offs] != 0 && buffer[offs - 1] != LF) {
            offs++;
        }
    }

    return true;
}

bool Sodaq_N2X::checkURC(char* buffer)
{
    if (buffer[0] != '+') {
        return false;
    }

    int param1, param2;

    if (sscanf(buffer, "+UFOTAS: %d,%d", &param1, &param2) == 2) {
        #ifdef DEBUG
        debugPrint("Unsolicited: FOTA: ");
        debugPrint(param1);
        debugPrint(", ");
        debugPrintln(param2);
        #endif

        return true;
    }

    if ((sscanf(buffer, "+NSONMI: %d,%d", &param1, &param2) == 2) && (param1 >= 0) && (param1 <= 6)) {
        debugPrint("Unsolicited: Socket ");
        debugPrint(param1);
        debugPrint(": ");
        debugPrintln(param2);

        if (param1 >= 0 && param1 < SOCKET_COUNT) {
            _socketPendingBytes[param1] = param2;
        }

        return true;
    }

    return false;
}

/**
 * 1. check echo
 * 2. check ok
 * 3. check error
 * 4. if response prefix is not empty, check response prefix, append if multiline
 * 5. check URC, if handled => continue
 * 6. if response prefis is empty, return the whole line return line buffer, append if multiline
*/
GSMResponseTypes Sodaq_N2X::readResponse(char* outBuffer, size_t outMaxSize, const char* prefix, uint32_t timeout)
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

void Sodaq_N2X::reboot()
{
    println("AT+NRB");

    // wait up to 2000ms for the modem to come up
    uint32_t start = millis();

    while ((readResponse() != GSMResponseOK) && !is_timedout(start, 2000)) {}
}

bool Sodaq_N2X::setNconfigParam(const char* param, const char* value)
{
    print("AT+NCONFIG=\"");
    print(param);
    print("\",\"");
    print(value);
    println('"');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_N2X::waitForSignalQuality(uint32_t timeout)
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

uint32_t Sodaq_N2X::convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec)
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

bool Sodaq_N2X::startsWith(const char* pre, const char* str)
{
    return (strncmp(pre, str, strlen(pre)) == 0);
}


/******************************************************************************
* Generic
*****************************************************************************/

// Returns true if the modem is on.
bool Sodaq_N2X::isOn() const
{
    if (_onoff) {
        return _onoff->isOn();
    }

    // No onoff. Let's assume it is on.
    return true;
}

void Sodaq_N2X::writeProlog()
{
    if (!_appendCommand) {
        debugPrint(">> ");
        _appendCommand = true;
    }
}

// Write a byte, as binary data
size_t Sodaq_N2X::writeByte(uint8_t value)
{
    return _modemStream->write(value);
}

size_t Sodaq_N2X::print(const String& buffer)
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_N2X::print(const char buffer[])
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_N2X::print(char value)
{
    writeProlog();
    debugPrint(value);

    return _modemStream->print(value);
};

size_t Sodaq_N2X::print(unsigned char value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_N2X::print(int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_N2X::print(unsigned int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_N2X::print(long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_N2X::print(unsigned long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_N2X::println(const __FlashStringHelper *ifsh)
{
    return print(ifsh) + println();
}

size_t Sodaq_N2X::println(const String &s)
{
    return print(s) + println();
}

size_t Sodaq_N2X::println(const char c[])
{
    return print(c) + println();
}

size_t Sodaq_N2X::println(char c)
{
    return print(c) + println();
}

size_t Sodaq_N2X::println(unsigned char b, int base)
{
    return print(b, base) + println();
}

size_t Sodaq_N2X::println(int num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_N2X::println(unsigned int num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_N2X::println(long num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_N2X::println(unsigned long num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_N2X::println(double num, int digits)
{
    writeProlog();
    debugPrint(num, digits);

    return _modemStream->println(num, digits);
}

size_t Sodaq_N2X::println(const Printable& x)
{
    return print(x) + println();
}

size_t Sodaq_N2X::println()
{
    debugPrintln();
    size_t i = print(CR);
    _appendCommand = false;
    return i;
}

// Initializes the input buffer and makes sure it is only initialized once.
// Safe to call multiple times.
void Sodaq_N2X::initBuffer()
{
    debugPrintln("[initBuffer]");

    // make sure the buffers are only initialized once
    if (!_isBufferInitialized) {
        _inputBuffer = static_cast<char*>(malloc(_inputBufferSize));
        _isBufferInitialized = true;
    }
}

// Sets the modem stream.
void Sodaq_N2X::setModemStream(Stream& stream)
{
    _modemStream = &stream;
}

// Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
int Sodaq_N2X::timedRead(uint32_t timeout) const
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
size_t Sodaq_N2X::readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout)
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
size_t Sodaq_N2X::readBytes(uint8_t* buffer, size_t length, uint32_t timeout)
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
size_t Sodaq_N2X::readLn(char* buffer, size_t size, uint32_t timeout)
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

Sodaq_SARA_N211_OnOff::Sodaq_SARA_N211_OnOff()
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

void Sodaq_SARA_N211_OnOff::on()
{
    #ifdef PIN_SARA_ENABLE
    digitalWrite(SARA_ENABLE, HIGH);
    digitalWrite(SARA_TX_ENABLE, HIGH);

    _onoff_status = true;
    #endif
}

void Sodaq_SARA_N211_OnOff::off()
{
    #ifdef PIN_SARA_ENABLE
    digitalWrite(SARA_ENABLE, LOW);
    digitalWrite(SARA_TX_ENABLE, LOW);

    // Should be instant
    // Let's wait a little, but not too long
    delay(50);
    _onoff_status = false;
    #endif
}

bool Sodaq_SARA_N211_OnOff::isOn()
{
    return _onoff_status;
}
