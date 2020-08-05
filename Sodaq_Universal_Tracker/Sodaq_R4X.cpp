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

//#define DEBUG

#define EPOCH_TIME_OFF          946684800  /* This is 1st January 2000, 00:00:00 in epoch time */
#define ATTACH_NEED_REBOOT      40000
#define REBOOT_DELAY            3500
#define REBOOT_TIMEOUT          15000
#define POWER_OFF_DELAY         5000

#define SODAQ_R4X_DEFAULT_CID           1

#define DEFAULT_CGACT_TIMEOUT           (150L * 1000)
#define DEFAULT_COPS_TIMEOUT            (180L * 1000)
#define DEFAULT_UMQTT_TIMEOUT           (60L * 1000)

/**
 * SARA R4X GPIO pin for network status indicator
 */
#define NETWORK_STATUS_GPIO_ID     16

#define DEBUG_STR_ERROR            "[ERROR]: "

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

/**
 * DIM is a define for the array size
 */
#define DIM(x)          (sizeof(x) / sizeof(x[0]))

#ifdef DEBUG
#define debugPrint(...)   { if (_diagPrint) _diagPrint->print(__VA_ARGS__); }
#define debugPrintln(...) { if (_diagPrint) _diagPrint->println(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrint(...)
#define debugPrintln(...)
#endif

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

Sodaq_R4X::Sodaq_R4X() : Sodaq_Ublox()
{
    _echoOff = false;
    _hexMode = false;
    _psm = false;
    _upsv = false;

    _mqttLoginResult     = -1;
    _mqttPendingMessages = -1;
    _mqttSubscribeReason = -1;
    _networkStatusLED    = 0;

    _pin = 0;
    _cid = SODAQ_R4X_DEFAULT_CID;
    _urat = SODAQ_R4X_DEFAULT_URAT;
    _opr = SODAQ_R4X_AUTOMATIC_OPERATOR;
    _mnoProfile = SIM_ICCID;
    _bandMaskLTE = BAND_MASK_UNCHANGED;
    _bandMaskNB = BAND_MASK_UNCHANGED;

    _httpGetHeaderSize = 0;

    /*
     * Assume all socket are closed
     */
    for (size_t ix = 0; ix < DIM(_socketClosed); ix++) {
        _socketClosed[ix] = true;
    }
    memset(_socketPendingBytes, 0, sizeof(_socketPendingBytes));

    _cgact_timeout = DEFAULT_CGACT_TIMEOUT;
    _cops_timeout = DEFAULT_COPS_TIMEOUT;
    _umqtt_timeout = DEFAULT_UMQTT_TIMEOUT;
}

// Initializes the modem instance. Sets the modem UART and the on-off power pins.
void Sodaq_R4X::init(Sodaq_OnOffBee* onoff, Uart& uart, uint32_t baud)
{
    debugPrintln("[init] started.");

    initBuffer(); // safe to call multiple times

    initUART(uart, baud);

    setOnOff(onoff);
}

/**
 * Turns the modem on and returns true if successful.
 *
 * With the _onoff we keep track of the power supply.
 * The following situations can occur:
 * 1. the modem is turned off from power (onoff power off)
 * 2. the modem is on (onoff power on), and reacts to AT commands
 * 3. the modem is on (onoff power on), but is in PSM (Power Safe Mode)
 *    In this situation it wakes up after the first AT command. So a second
 *    AT command is needed to be sure that it responds to it.
 * 4. the modem is on (onoff power on), but it does not respond to AT
 *    commands
 */
bool Sodaq_R4X::on()
{
    debugPrintln("[R4X on]");
    if (!_onoff) {
        debugPrintln("[R4X on] Missing _onoff");
        return false;
    }

    bool need_power_toggle = false;
    bool was_off = !isOn();

    _startOn = millis();

    if (isOn()) {
        if (isAlive(1)) {
            return true;
        }

        /* Even if we think it is on, the modem could have gone into
         * Power Safe Mode.
         * In that case it needs to be toggled on.
         */
        need_power_toggle = true;
    }

    if (!isOn() || need_power_toggle) {
        _onoff->on();
    }

    if (was_off) {
        uint32_t baud = determineBaudRate(_baudRate);
        if (baud == 0) {
            debugPrintln("ERROR: No Reply from Modem");
            return false;
        }

        // Extra read just to clear the input stream
        readResponse(NULL, 0, NULL, 250);

        if (_baudRate != baud) {
            String cmd = String("AT+IPR=") + _baudRate;
            if (execCommand(cmd)) {
                _modemUART->begin(_baudRate);
                /*
                 * Wait at least 100 ms before issuing a new AT command
                 */
                sodaq_wdt_safe_delay(110);

                if (!isAlive(9)) {
                    debugPrintln("ERROR: No Reply from modem after baudrate switch");
                    return false;
                }
            }
            else {
                return false;
            }
        }

        execCommand("AT+CEDRXS=0");
        if (_psm) {
            execCommand("AT+CPSMS=1");
        }
        else {
            execCommand("AT+CPSMS=0");
        }

        if (_upsv) {
            execCommand("AT+UPSV=4");
        }
        else {
            execCommand("AT+UPSV=0");
        }

        execCommand("AT+CPSMS?");
        execCommand("AT+CEDRXS?");
        execCommand("AT+UPSV?");
    }

    return isOn();
}

// Turns the modem off and returns true if successful.
bool Sodaq_R4X::off()
{
    debugPrintln("[R4X off]");
    if (!_onoff) {
        debugPrintln("[R4X off] Missing _onoff");
        return false;
    }

    // Safety command to shutdown, response is ignored
    if (isOn()) {
        println("AT+CPWROFF");
        readResponse(NULL, 0, NULL, 1000);
        sodaq_wdt_safe_delay(POWER_OFF_DELAY);
    }

    // No matter if it is on or off, turn it off.
    _onoff->off();

    _echoOff = false;
    _hexMode = false;
    _mqttLoginResult = -1;

    debugPrintln("[R4X off, off]");
    return !isOn();
}

/**
 * Get the nth valid baudrate
 *
 * The R4X supports 9600, 19200, 38400, 57600, 115200 (default and factory-
 * programmed value), 230400, 460800
 */
uint32_t Sodaq_R4X::getNthValidBaudRate(size_t nth)
{
    uint32_t baud_rates[] = {
            115200,
            230400,
            460800,
            9600,
            19200,
            38400,
            57600,
    };
    if (nth >= DIM(baud_rates)) {
        return 0;
    }
    return baud_rates[nth];
}

void Sodaq_R4X::switchEchoOff()
{
    if (!_echoOff) {
        // Suppress echoing
        const size_t retry_count = 3;
        for (size_t ix = 0; ix < retry_count; ix++) {
            if (execCommand("ATE0")) {
                _echoOff = true;
                break;
            }
        }
    }
}

/**
 * Enable HEX mode
 */
bool Sodaq_R4X::enableHexMode()
{
    if (!_hexMode) {
        const size_t retry_count = 3;
        for (size_t ix = 0; ix < retry_count; ix++) {
            if (execCommand("AT+UDCONF=1,1")) {
                _hexMode = true;
                break;
            }
        }
        if (!_hexMode) {
            debugPrintln("[R4X] ERROR: Failed to set HEX mode");
        }
    }
    return _hexMode;
}

bool Sodaq_R4X::connect(const char* apn, const char* urat, MNOProfile mnoProfile,
    const char* opr, const char* bandMaskLTE, const char* bandMaskNB)
{
    setApn(apn);
    setUrat(urat);
    if (opr == 0) {
        opr = SODAQ_R4X_AUTOMATIC_OPERATOR;
    }
    setMnoProfile(mnoProfile);
    setOperator(opr);
    setBandMaskLTE(bandMaskLTE);
    setBandMaskNB(bandMaskNB);
    return connect();
}

bool Sodaq_R4X::connect(const char* apn, const char* urat, const char* bandMask)
{
    return connect(apn, urat, SIM_ICCID, SODAQ_R4X_AUTOMATIC_OPERATOR, BAND_MASK_UNCHANGED, bandMask);
}

bool Sodaq_R4X::connect()
{
    debugPrintln("[R4X connect]");

    uint32_t start_ts = millis();
    uint32_t remaining_timeout;

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

    switchEchoOff();

    if (!checkCFUN()) {
        return false;
    }

    if (!checkMnoProfile(_mnoProfile)) {
        return false;
    }

    if (_urat == 0) {
        _urat = SODAQ_R4X_DEFAULT_URAT;
    }
    if (!checkUrat(_urat)) {
        return false;
    }

    if (!checkBandMasks(_bandMaskLTE, _bandMaskNB)) {
        return false;
    }

    size_t cops_retry_count = 3;
    bool cops_succeeded = false;
    for (size_t i = 0; i < cops_retry_count && !is_timedout(start_ts, _connect_timeout); i++) {
        if (checkCOPS(_opr, _urat)) {
            cops_succeeded = true;
            break;
        }
        sodaq_wdt_safe_delay(1000);
    }
    if (!cops_succeeded) {
        return false;
    }

    if (is_timedout(start_ts, _connect_timeout)) {
        return false;
    }

    if (!checkApn(_apn)) {
        return false;
    }
    remaining_timeout = _connect_timeout - (millis() - start_ts);
    if (is_timedout(start_ts, _connect_timeout)) {
        return false;
    }

    uint32_t tm = millis();

    if (!waitForSignalQuality(remaining_timeout)) {
        return false;
    }
    if (is_timedout(start_ts, _connect_timeout)) {
        return false;
    }

    remaining_timeout = _connect_timeout - (millis() - start_ts);
    if (!attachGprs(remaining_timeout)) {
        return false;
    }

    if (millis() - tm > ATTACH_NEED_REBOOT) {
        /* TODO
         * It is a bit weird that this section is not looking at
         * the _connect_timeout.
         */
        reboot();
        
        if (!waitForSignalQuality()) {
            return false;
        }

        if (!attachGprs()) {
            return false;
        }
    }

    execCommand("AT+CEDRXS=0");
    if (_psm) {
        execCommand("AT+CPSMS=1");
    }
    else {
        execCommand("AT+CPSMS=0");
    }

    if (_upsv) {
        execCommand("AT+UPSV=4");
    }
    else {
        execCommand("AT+UPSV=0");
    }

    execCommand("AT+CPSMS?");
    execCommand("AT+CEDRXS?");
    execCommand("AT+UPSV?");

    return doSIMcheck();
}

// Disconnects the modem from the network.
bool Sodaq_R4X::disconnect()
{
    return execCommand("AT+CGATT=0", _disconnect_timeout);
}


/******************************************************************************
* Public
*****************************************************************************/

bool Sodaq_R4X::attachGprs(uint32_t timeout)
{
    debugPrintln("[R4X attachGprs]");

    uint32_t start = millis();
    uint32_t delay_count = 500;
    bool retval = false;

    while (!is_timedout(start, timeout)) {
        if (isAttached()) {
            /*
             * Don't use more timeout than we're being told.
             */
            uint32_t my_timeout = _cgact_timeout;
            if (my_timeout > timeout) {
                my_timeout = timeout;
            }
            if (isDefinedIP4() || (execCommand("AT+CGACT=1", my_timeout) && isDefinedIP4())) {
                retval = true;
                break;
            }
        }

        sodaq_wdt_safe_delay(delay_count);

        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 1000;
        }
    }

    if (is_timedout(start, timeout)) {
        debugPrintln("[R4X attachGprs] timed out");
    }
    return retval;
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

bool Sodaq_R4X::getOperatorInfo(uint16_t* mcc, uint16_t* mnc)
{
    // Set mode to numeric format
    if (!execCommand("AT+COPS=3,2")) {
        return false;
    }

    println("AT+COPS?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    unsigned long operatorCode = 0;

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+COPS: ") == GSMResponseOK) &&
        (strlen(responseBuffer) > 0)) {
        // Expect something like this: 0,0,"dddddd"
        // 5 or 6 characters long for numeric format (MCC/MNC codes)
        if (sscanf(responseBuffer, "%*d,%*d,\"%lu\"", &operatorCode) == 1) {
            uint16_t divider = (operatorCode > 100000) ? 1000 : 100;

            *mcc = operatorCode / divider;
            *mnc = operatorCode % divider;

            return true;
        }
    }

    return false;
}

bool Sodaq_R4X::getOperatorInfoNumber(char* buffer, size_t size)
{
    buffer[0] = 0;

    // Set mode to numeric format
    if (!execCommand("AT+COPS=3,2")) {
        return false;
    }

    return getOperatorInfo_low(buffer, size);
}

bool Sodaq_R4X::getOperatorInfoString(char* buffer, size_t size)
{
    buffer[0] = 0;

    // Set mode to long alphanumeric format
    if (!execCommand("AT+COPS=3,0")) {
        return false;
    }

    return getOperatorInfo_low(buffer, size);
}

bool Sodaq_R4X::getOperatorInfo_low(char* buffer, size_t size)
{
    // may be up to 24 characters long for long alphanumeric format
    if (size < 24 + 1) {
         return false;
    }

    println("AT+COPS?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+COPS: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {
        // Let's hope for the best, that we don't overflow the buffer
        if (sscanf(responseBuffer, "%*d,%*d,\"%[^\"]\"", buffer) == 1) {
            return true;
        }
    }

    return false;
}

bool Sodaq_R4X::getCellInfo(uint16_t* tac, uint32_t* cid, uint16_t* urat)
{
    // 2: network registration and location information URC +CEREG: <stat>[,[<tac>],[<ci>],[<AcT>]] enabled
    if (!execCommand("AT+CEREG=2")) {
        return false;
    }

    println("AT+CEREG?");

    char responseBuffer[64];
    memset(responseBuffer, 0, sizeof(responseBuffer));

    if ((readResponse(responseBuffer, sizeof(responseBuffer), "+CEREG: ") == GSMResponseOK) && (strlen(responseBuffer) > 0)) {
        // Expect something like: 2,5,"EBF5","141E10D",7
        // <tac> Two bytes tracking area code in hexadecimal format
        // <ci> Four bytes E-UTRAN cell-id in hexadecimal format
        // <AcT> 7: E-UTRAN, 8: E-UTRAN EC-GSM-IoT, 9: E-UTRAN Cat NB1
        short unsigned int num1;
        unsigned int num2;
        short int num3;
        if (sscanf(responseBuffer, "2,%*d,\"%hx\",\"%x\",%hi", &num1, &num2, &num3) == 3) {
            *tac = num1;
            *cid = num2;
            *urat = num3;
            switch (*urat) {
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

                if (sscanf(responseBuffer, "2,%*d,\"%hx\",\"%x\"", &num1, &num2) == 2) {
                    *tac = num1;
                    *cid = num2;
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

// Gets International Mobile Equipment Identity.
// Should be provided with a buffer of at least 16 bytes.
// Returns true if successful.
bool Sodaq_R4X::getIMEI(char* buffer, size_t size)
{
    if (buffer == NULL || size < 15 + 1) {
        return false;
    }

    return (execCommand("AT+CGSN", buffer, size) == GSMResponseOK) && (strlen(buffer) > 0) && (atoll(buffer) > 0);
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

/**
 * Is the modem still connected?
 *
 * Return true if the modem is connected to the network and IP address is not 0.0.0.0.
 */
bool Sodaq_R4X::isConnected()
{
    debugPrintln("[R4X isConnected]");
    /* TODO
     * There are three calls, each could timeout.
     * Why do we want to wait for CSQ here?
     */
    return isAttached() && waitForSignalQuality(10000) && isDefinedIP4();
}

// Returns true if defined IP4 address is not 0.0.0.0.
bool Sodaq_R4X::isDefinedIP4()
{
    char buffer[256];

    // FIXME Use AT+CGPADDR=1. That is, if we are sure we use cid (context identifier) 1.
    println("AT+CGDCONT?");
    // Expect response like this (already skipped "+CGDCONT: "):
    //     1,"IP","data.mono","10.140.4.195",0,0,0,0
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

bool Sodaq_R4X::loadApn(const char* apn)
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
    println(on ? '1' : '0');

    return (readResponse() == GSMResponseOK);
}

bool Sodaq_R4X::setVerboseErrors(bool on)
{
    print("AT+CMEE=");
    println(on ? '2' : '0');

    return (readResponse() == GSMResponseOK);
}

/******************************************************************************
* Sockets
*****************************************************************************/

bool Sodaq_R4X::socketClose(uint8_t socketID, bool async)
{
    (void)socketFlush(socketID);

    print("AT+USOCL=");
    print(socketID);

    if (async) {
        println(",1");
    }
    else {
        println();
    }

    _socketClosed[socketID] = true;
    _socketPendingBytes[socketID] = 0;

    if (readResponse(NULL, 0, NULL, _socket_close_timeout) != GSMResponseOK) {
        (void)execCommand(String("AT+USOCTL=") + socketID + ",1");
        (void)execCommand("AT+USOER");
        return false;
    }

    return true;
}

bool Sodaq_R4X::socketConnect(uint8_t socketID, const char* remoteHost, const uint16_t remotePort)
{
    print("AT+USOCO=");
    print(socketID);
    print(",\"");
    print(remoteHost);
    print("\",");
    println(remotePort);

    bool b = readResponse(NULL, 0, NULL, _socket_connect_timeout) == GSMResponseOK;

    _socketClosed[socketID] = !b;

    return b;
}

/**
 * Create a new socket
 *
 * @returns 0..(SOCKET_COUNT-1) for a successful creation
 * @returns -1 if creation failed
 */
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
        return -1;
    }

    int socketID;

    if ((sscanf(buffer, "%d", &socketID) != 1) || (socketID < 0) || (socketID > SODAQ_UBLOX_SOCKET_COUNT)) {
        return -1;
    }

    /* Start with socket closed
     */
    _socketClosed[socketID] = true;
    _socketPendingBytes[socketID] = 0;

    return socketID;
}

/**
 * Flush the bytes in the send buffer of a TCP socket
 *
 * \returns true all bytes flushed
 * \returns true if socket closed
 * \returns false if wrong answer from AT+USOCTL
 * \returns false if timed out
 */
bool Sodaq_R4X::socketFlush(uint8_t socketID, uint32_t timeout)
{
    uint32_t start = millis();

    while (!is_timedout(start, timeout) && !_socketClosed[socketID]) {
        print("AT+USOCTL=");
        print(socketID);
        println(",11");
        
        char buffer[32];
        if (readResponse(buffer, sizeof(buffer), "+USOCTL: ") != GSMResponseOK) {
            /* We did not get an OK.
             * If the socket was closed then assume the flush went fine.
             */
            if (_socketClosed[socketID]) {
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

        /*
         * How often do we want to repeat?
         */
        sodaq_wdt_safe_delay(150);
    }

    /* FIXME In case the socket is closed or the modem did not respond anymore
     * then assume the flush went fine.
     */
    if (_socketClosed[socketID]) {
        return true;
    }

    return false;
}

bool Sodaq_R4X::socketIsClosed(uint8_t socketID)
{
    return _socketClosed[socketID];
}

/*
 * Read a buffer from a TCP socket
 */
size_t Sodaq_R4X::socketRead(uint8_t socketID, uint8_t* buffer, size_t size)
{
    if (!socketHasPendingBytes(socketID)) {
        // no URC has happened, no socket to read
        debugPrintln("[R4X] ERROR: Reading from socket without bytes available");
        return 0;
    }

    if (!enableHexMode()) {
        return 0;
    }

    /* Determine the size we can handle.
     * This could mean we read less bytes then there are available.
     * TODO We can only handle SODAQ_R4X_MAX_SOCKET_BUFFER / 2 (2 chars per byte).
     */
    size = min(size, min(SODAQ_R4X_MAX_SOCKET_BUFFER, _socketPendingBytes[socketID]));

    char   outBuffer[SODAQ_R4X_MAX_SOCKET_BUFFER];      // WARNING! Large allocation on stack
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

    /* TODO Maybe it is better to check socketID == retSocketID
     */
    if ((retSocketID < 0) || (retSocketID >= SODAQ_UBLOX_SOCKET_COUNT)) {
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

/**
 * Read a buffer from a UDP socket
 */
size_t Sodaq_R4X::socketReceive(uint8_t socketID, uint8_t* buffer, size_t size)
{
    if (!socketHasPendingBytes(socketID)) {
        // no URC has happened, no socket to read
        debugPrintln("[R4X] ERROR: Reading from socket without bytes available");
        return 0;
    }

    /* Enable HEX mode
     */
    if (!enableHexMode()) {
        return 0;
    }

    /* Determine the size we can handle.
     * This could mean we read less bytes then there are available.
     * TODO We can only handle SODAQ_R4X_MAX_SOCKET_BUFFER / 2 (2 chars per byte).
     */
    size = min(size, min(SODAQ_R4X_MAX_SOCKET_BUFFER, _socketPendingBytes[socketID]));

    char   outBuffer[SODAQ_R4X_MAX_SOCKET_BUFFER];      // WARNING! Large allocation on stack
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

    /* TODO Maybe it is better to check socketID == retSocketID
     */
    if ((retSocketID < 0) || (retSocketID >= SODAQ_UBLOX_SOCKET_COUNT)) {
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

/**
 * Write a buffer to a UDP socket
 */
size_t Sodaq_R4X::socketSend(uint8_t socketID, const char* remoteHost, const uint16_t remotePort,
                             const uint8_t* buffer, size_t size)
{
    if (size > SODAQ_MAX_SEND_MESSAGE_SIZE) {
        debugPrintln("[R4X] ERROR: Message exceeded maximum size!");
        return 0;
    }

    if (!enableHexMode()) {
        return 0;
    }

    /* Show the socket error
     * Just is just for diagnostics. Somehow we always see error 65
     * after doing a USOST. 65 is EEOF - End of file.
     */
    (void)execCommand(String("AT+USOCTL=") + socketID + ",1");

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
    if (readResponse(outBuffer, sizeof(outBuffer), "+USOST: ", _socket_write_timeout) != GSMResponseOK) {
        return 0;
    }

    int retSocketID;
    int sentLength;
    if ((sscanf(outBuffer, "%d,%d", &retSocketID, &sentLength) != 2) || (retSocketID != socketID)) {
        /* Wrong socket or unexpected other result */
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

/**
 * Are there pending bytes on the TCP socket?
 */
bool Sodaq_R4X::socketWaitForRead(uint8_t socketID, uint32_t timeout)
{
    if (socketHasPendingBytes(socketID)) {
        return true;
    }

    /* Query the modem again if there are pending bytes.
     */
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

/**
 * Are there pending bytes on the UDP socket?
 */
bool Sodaq_R4X::socketWaitForReceive(uint8_t socketID, uint32_t timeout)
{
    if (socketHasPendingBytes(socketID)) {
        return true;
    }

    /* Query the modem again if there are pending bytes.
     */
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

/**
 * Write a buffer to a TCP socket
 */
size_t Sodaq_R4X::socketWrite(uint8_t socketID, const uint8_t* buffer, size_t size)
{
    if (!enableHexMode()) {
        return 0;
    }

    print("AT+USOWR=");
    print(socketID);
    print(",");
    println(size);

    /* Wait for the prompt. It should come in immediately.
     */
    if (!waitForSocketPrompt(100)) {
        return 0;
    }

    // After the @ prompt reception, wait for a minimum of 50 ms before sending data.
    delay(51);

    for (size_t i = 0; i < size; i++) {
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(buffer[i]))));
        print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(buffer[i]))));
    }
    /* Indicate that we need a new diag prolog (">>") next time we send a command
     */
    dbprintln();

    char outBuffer[64];
    if (readResponse(outBuffer, sizeof(outBuffer), "+USOWR: ", _socket_write_timeout) != GSMResponseOK) {
        return 0;
    }

    int retSocketID;
    int sentLength;
    if ((sscanf(outBuffer, "%d,%d", &retSocketID, &sentLength) != 2) ||
        (retSocketID < 0) ||
        (retSocketID > SODAQ_UBLOX_SOCKET_COUNT)) {
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

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", _umqtt_timeout) == GSMResponseOK) && startsWith("0,1", buffer);
}

void Sodaq_R4X::mqttLoop()
{
    sodaq_wdt_reset();
    if (_modemUART->available()) {

        int count = readLn(250);        // 250ms, how many bytes at which baudrate?

        if (count > 0) {
            debugPrint("<< ");
            debugPrintln(getInputBuffer());

            checkURC(getInputBuffer());
        }
    }
}

bool Sodaq_R4X::mqttPing(const char* server)
{
    char buffer[16];

    print("AT+UMQTTC=8,\"");
    print(server);
    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", _umqtt_timeout) == GSMResponseOK) && startsWith("8,1", buffer);
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

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", _umqtt_timeout) == GSMResponseOK) && startsWith("2,1", buffer);
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

    if ((readResponse(bufferIn, sizeof(bufferIn), "+UMQTTC: ", _umqtt_timeout) != GSMResponseOK) || !startsWith("6,1", bufferIn)) {
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
        int count = readLn(250);
        sodaq_wdt_reset();

        if (count <= 0) {
            continue;
        }

        bool isTopic = startsWith("Topic:", getInputBuffer());
        bool isMessage = startsWith("Msg:", getInputBuffer());

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

            memcpy(buffer + outSize, getInputBuffer() + headerSize, count);
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

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("4,1", buffer);
}

bool Sodaq_R4X::mqttSetCleanSession(bool enabled)
{
    char buffer[16];

    print("AT+UMQTT=12,");
    println(enabled ? '1' : '0');

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("12,1", buffer);
}

bool Sodaq_R4X::mqttSetClientId(const char* id)
{
    char buffer[16];

    print("AT+UMQTT=0,\"");
    print(id);
    println('"');

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("0,1", buffer);
}

bool Sodaq_R4X::mqttSetInactivityTimeout(uint16_t timeout)
{
    char buffer[16];

    print("AT+UMQTT=10,");
    println(timeout);

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("10,1", buffer);
}

bool Sodaq_R4X::mqttSetLocalPort(uint16_t port)
{
    char buffer[16];

    print("AT+UMQTT=1,");
    println(port);

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("1,1", buffer);
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

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("11,1", buffer);
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

  return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("2,1", buffer);
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

    return (readResponse(buffer, sizeof(buffer), "+UMQTT: ", _umqtt_timeout) == GSMResponseOK) && startsWith("3,1", buffer);
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

    if ((readResponse(buffer, sizeof(buffer), "+UMQTTC: ", _umqtt_timeout) != GSMResponseOK) || !startsWith("4,1", buffer)) {
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

    return (readResponse(buffer, sizeof(buffer), "+UMQTTC: ", _umqtt_timeout) == GSMResponseOK) && startsWith("5,1", buffer);
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

    char reply_buffer[256];       // Filenames can be 248 chars long

    // Read reply identifier +URDFILE:
    size_t len = readBytesUntil(' ', reply_buffer, sizeof(reply_buffer));
    if (len == 0 || strstr(reply_buffer, "+URDFILE:") == NULL) {
        debugPrintln(DEBUG_STR_ERROR "+URDFILE literal is missing!");
        return 0;
    }

    // Read filename
    len = readBytesUntil(',', reply_buffer, sizeof(reply_buffer));
    // TODO check filename after removing quotes and escaping chars

    // Read filesize
    len = readBytesUntil(',', reply_buffer, sizeof(reply_buffer));
    filesize = 0; // reset the var before reading from reply string
    if (sscanf(reply_buffer, "%lu", &filesize) != 1) {
        debugPrintln(DEBUG_STR_ERROR "Could not parse the file size!");
        return 0;
    }
    /* Filesize was already checked. Repeat it once more.
     */
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

    char reply_buffer[256];       // Filenames can be 248 chars long

    // Read reply identifier
    //   +URDBLOCK: http_last_response_0,86,"..."
    // where 86 is an example of the size
    size_t len = readBytesUntil(' ', reply_buffer, sizeof(reply_buffer));
    if (len == 0 || strstr(reply_buffer, "+URDBLOCK:") == NULL) {
        debugPrintln(DEBUG_STR_ERROR "+URDBLOCK literal is missing!");
        return 0;
    }

    // skip filename
    len = readBytesUntil(',', reply_buffer, sizeof(reply_buffer));
    // TODO check filename. Note, there are no quotes. ??Manual example has qotes

    // read the number of bytes
    len = readBytesUntil(',', reply_buffer, sizeof(reply_buffer));
    uint32_t blocksize = 0; // reset the var before reading from reply string
    if (sscanf(reply_buffer, "%lu", &blocksize) != 1) {
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

    if (waitForFilePrompt(250)) {
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

/**
 * Check APN
 *
 */
bool Sodaq_R4X::checkApn(const char* requiredAPN)
{
    char buffer[256];
    println("AT+CGDCONT?");
    // Expect response like this (already skipped "+CGDCONT: "):
    //     1,"IP","data.mono","10.140.4.195",0,0,0,0
    if (readResponse(buffer, sizeof(buffer), "+CGDCONT: ") != GSMResponseOK) {
        return false;
    }

    if (strncmp(buffer, "1,\"IP\"", 6) == 0 && strncmp(buffer + 6, ",\"\"", 3) != 0) {
        char apn[64];

        if (sscanf(buffer + 6, ",\"%[^\"]\"", apn) != 1) {
            return false;
        }

        if (strcmp(apn, requiredAPN) == 0) {
            return true;
        }
    }

    return loadApn(requiredAPN);
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

    if (!setRadioActive(false)) {
        return false;
    }

    if (setLTEMask) {
        print("AT+UBANDMASK=");
        print("0,");
        println(bandMaskLTE);
        if (readResponse() != GSMResponseOK) {
            return false;
        }
    }

    if (setNBMask) {
        print("AT+UBANDMASK=");
        print("1,");
        println(bandMaskNB);
        if (readResponse() != GSMResponseOK) {
            return false;
        }
    }

    /* This saves the current settings to the NVM, then it reboots the module
     */
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
    if ((strcmp(requiredOperator, SODAQ_R4X_AUTOMATIC_OPERATOR) == 0) &&
            (strcmp(requiredURAT, SODAQ_R4X_NBIOT_URAT) != 0)){
        return execCommand("AT+COPS=0,2", _cops_timeout);
    }

    println("AT+COPS=3,2");
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    println("AT+COPS?");

    char buffer[64];

    if (readResponse(buffer, sizeof(buffer), "+COPS: ", _cops_timeout) != GSMResponseOK) {
        return false;
    }

    if (strcmp(requiredOperator, SODAQ_R4X_AUTOMATIC_OPERATOR) == 0) {
        return ((strncmp(buffer, "0", 1) == 0) || execCommand("AT+COPS=0,2", _cops_timeout));
    }
    else if ((strncmp(buffer, "1", 1) == 0) &&
            (strncmp(buffer + 5, requiredOperator, strlen(requiredOperator)) == 0)) {
        return true;
    }
    else {
        print("AT+COPS=1,2,\"");
        print(requiredOperator);
        println('"');

        return (readResponse(NULL, 0, NULL, _cops_timeout) == GSMResponseOK);
    }
}

/**
 * Check and set the MNO (Mobile Network Operator) profile
 *
 * The manual describes it as follows:
 * "Follow this procedure to properly set up the configuration:
 *  • Deregister the module from the network (perform a AT+CFUN=0 or
 *    AT+CFUN=4 cycle or issue the AT+COPS=2 command)
 *  • Issue AT+UMNOPROF=<MNO>
 *  • Reboot the module (AT+CFUN=15) in order to apply the new configuration
 * However, it was pointed out by Ublox that the first step MUST be
 * a AT+CFUN=0, setRadioActive(false). This is also true for AT+URAT= and
 * AT+UBANDMASK=
 */
bool Sodaq_R4X::checkMnoProfile(MNOProfile requiredProfile)
{
    println("AT+UMNOPROF?");

    char buffer[64];
    if (readResponse(buffer, sizeof(buffer), "+UMNOPROF: ") != GSMResponseOK) {
        return false;
    }

    if (atoi(buffer) == requiredProfile) {
        return true;
    }

    if (!setRadioActive(false)) {
        return false;
    }

    print("AT+UMNOPROF=");
    println(requiredProfile);
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    /* This saves the current settings to the NVM, then it reboots the module
     */
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

    if (!setRadioActive(false)) {
        return false;
    }

    print("AT+URAT=");
    println(requiredURAT);
    if (readResponse() != GSMResponseOK) {
        return false;
    }

    /* This saves the current settings to the NVM, then it reboots the module
     */
    reboot();

    return true;
}

bool Sodaq_R4X::checkURC(const char* buffer)
{
    if (buffer[0] != '+') {
        return false;
    }

    int param1;
    int param2;
    char param3[1024];                                  // WARNING! Large allocation on stack

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

        if (param1 >= 0 && param1 < SODAQ_UBLOX_SOCKET_COUNT) {
            _socketPendingBytes[param1] = param2;
        }

        return true;
    }

    if (sscanf(buffer, "+UUSORF: %d,%d", &param1, &param2) == 2) {
        debugPrint("Unsolicited: Socket ");
        debugPrint(param1);
        debugPrint(": ");
        debugPrintln(param2);

        if (param1 >= 0 && param1 < SODAQ_UBLOX_SOCKET_COUNT) {
            _socketPendingBytes[param1] = param2;
        }

        return true;
    }

    if (sscanf(buffer, "+UUSOCL: %d", &param1) == 1) {
        debugPrint("Unsolicited: Socket ");
        debugPrintln(param1);

        if (param1 >= 0 && param1 < SODAQ_UBLOX_SOCKET_COUNT) {
            _socketClosed[param1] = true;
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

void Sodaq_R4X::reboot()
{
    debugPrintln("[reboot]");

    execCommand("AT+CFUN=15");
    _echoOff = false;

    // wait for the reboot to start
    sodaq_wdt_safe_delay(REBOOT_DELAY);

    // wait a while for the modem to come up
    uint32_t start = millis();
    while (!isAlive() && !is_timedout(start, 3000)) {
        sodaq_wdt_safe_delay(100);
    }
    /* Flush any input that came in right after the loop timed out.
     */
    while (readLn(10) > 0) {
    }

    // echo off again after reboot
    switchEchoOff();

    while (!is_timedout(start, REBOOT_TIMEOUT)) {
        if (getSimStatus() == SimReady) {
            break;
        }
        sodaq_wdt_safe_delay(100);
    }
}

bool Sodaq_R4X::setSimPin(const char* simPin)
{
    print("AT+CPIN=\"");
    print(simPin);
    println('"');

    return (readResponse() == GSMResponseOK);
}

/******************************************************************************
* Generic
*****************************************************************************/

/**
 * Wait for the Socket Prompt '@'
 */
bool Sodaq_R4X::waitForSocketPrompt(uint32_t timeout)
{
    return waitForPrompt(STR_RESPONSE_SOCKET_PROMPT, timeout);
}

/**
 * Wait for the Socket Prompt '>'
 */
bool Sodaq_R4X::waitForFilePrompt(uint32_t timeout)
{
    return waitForPrompt(STR_RESPONSE_FILE_PROMPT, timeout);
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

    pinMode(SARA_R4XX_TOGGLE, INPUT);
    digitalWrite(SARA_R4XX_TOGGLE, HIGH);
#endif

    _onoff_status = false;
}

void Sodaq_SARA_R4XX_OnOff::on()
{
#ifdef PIN_SARA_ENABLE
    digitalWrite(SARA_ENABLE, HIGH);
    digitalWrite(SARA_TX_ENABLE, HIGH);

    /*
     * This is the PWR_ON pin of the R4X
     * See SARA-R4 series-Data sheet
     *  - turn low (between 0.15s and 3.2s) to trigger module switch on from power off mode
     *  - turn low (between 0.15s and 3.2s) to trigger module wake-up from PSM deep sleep
     *  - turn low (at least 1.5s) to trigger module switch off
     * Since we put on the power (SARA_ENABLE controls power supply), we
     * know it must be in power off mode.
     */
    pinMode(SARA_R4XX_TOGGLE, OUTPUT);
    digitalWrite(SARA_R4XX_TOGGLE, LOW);
    // TODO Can we reduce this to something like 200ms?
    sodaq_wdt_safe_delay(2000);

    /*
     * Make the pin INPUT and thus it will go HIGH due to the pull-up
     * in the R4X.
     */
    pinMode(SARA_R4XX_TOGGLE, INPUT);
#endif

    _onoff_status = true;
}

void Sodaq_SARA_R4XX_OnOff::off()
{
#ifdef PIN_SARA_ENABLE
    /*
     * Completely switch off power of the R4X.
     */
    digitalWrite(SARA_ENABLE, LOW);
    digitalWrite(SARA_TX_ENABLE, LOW);
#endif

    _onoff_status = false;
}

bool Sodaq_SARA_R4XX_OnOff::isOn()
{
    return _onoff_status;
}
