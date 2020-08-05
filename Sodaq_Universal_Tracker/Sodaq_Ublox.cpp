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

#include <time.h>
#include "Sodaq_wdt.h"

#include "Sodaq_Ublox.h"

#define DEBUG

#define SODAQ_UBLOX_DEFAULT_INPUT_BUFFER_SIZE   1024

#define DEFAULT_CONNECT_TIMEOUT         (10L * 60L * 1000)
#define DEFAULT_DISCONNECT_TIMEOUT      (40L * 1000)
#define DEFAULT_SOCKET_CLOSE_TIMEOUT    (120L * 1000)
#define DEFAULT_SOCKET_CONNECT_TIMEOUT  (120L * 1000)
#define DEFAULT_SOCKET_WRITE_TIMEOUT    (120L * 1000)

#define SODAQ_UBLOX_TERMINATOR          "\r\n"
#define SODAQ_UBLOX_TERMINATOR_LEN      (sizeof(SODAQ_UBLOX_TERMINATOR) - 1)

#define EPOCH_TIME_YEAR_OFF     100        /* years since 1900 */

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

Sodaq_Ublox::Sodaq_Ublox()
{
    _modemUART = 0;
    _baudRate = 0;
    _onoff = 0;

    _CSQtime = 0;
    _lastRSSI            = 0;
    _minRSSI             = -113;  // dBm

    _startOn = 0;
    _apn = 0;
    _cid = 0;

    _connect_timeout = DEFAULT_CONNECT_TIMEOUT;
    _disconnect_timeout = DEFAULT_DISCONNECT_TIMEOUT;

    _socket_close_timeout = DEFAULT_SOCKET_CLOSE_TIMEOUT;
    _socket_connect_timeout = DEFAULT_SOCKET_CONNECT_TIMEOUT;
    _socket_write_timeout = DEFAULT_SOCKET_WRITE_TIMEOUT;

    _isBufferInitialized = false;
    _inputBuffer         = 0;
    _inputBufferSize     = SODAQ_UBLOX_DEFAULT_INPUT_BUFFER_SIZE;

    _diagPrint = 0;
    _appendCommand = false;
}

Sodaq_Ublox::~Sodaq_Ublox()
{
}

// Initializes the input buffer and makes sure it is only initialized once.
// Safe to call multiple times.
void Sodaq_Ublox::initBuffer()
{
    debugPrintln("[initBuffer]");

    // make sure the buffers are only initialized once
    if (!_isBufferInitialized) {
        _inputBuffer = static_cast<char*>(malloc(_inputBufferSize));
        _isBufferInitialized = true;
    }
}

// Returns true if the modem is on.
bool Sodaq_Ublox::isOn() const
{
    if (_onoff) {
        return _onoff->isOn();
    }

    // No onoff. Let's assume it is on.
    return true;
}

// Returns true if the modem replies to "AT" commands without timing out.
bool Sodaq_Ublox::isAlive(size_t retry_count)
{
    bool retval = false;
    do {
        if (execCommand("AT", 450)) {
            retval = true;
            break;
        }
        if (retry_count > 0) {
            retry_count--;
        }
    } while (retry_count > 0);
    return retval;
}

/**
 * Determine the current baudrate
 *
 * The modem is expected to be on. Try from a list of baudrates
 * until an "OK" is read from the modem.
 */
uint32_t Sodaq_Ublox::determineBaudRate(uint32_t current)
{
    uint32_t baud;
    const uint8_t retry_count = 4;

    /* Try current baudrate first
    */
    if (current != 0) {
        if (isAlive(retry_count)) {
            return current;
        }
    }
    for (size_t ix = 0; ; ix++) {
        baud = getNthValidBaudRate(ix);
        if (baud == 0) {
            break;
        }
        _modemUART->begin(baud);
        if (isAlive(retry_count)) {
            return baud;
        }
    }

    return 0;
}

// Gets Integrated Circuit Card ID.
// Should be provided with a buffer of at least 21 bytes.
// Returns true if successful.
bool Sodaq_Ublox::getCCID(char* buffer, size_t size)
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
bool Sodaq_Ublox::getIMSI(char* buffer, size_t size)
{
    if (buffer == NULL || size < 20 + 1) {
        return false;
    }

    return (execCommand("AT+CIMI", buffer, size) == GSMResponseOK) && (strlen(buffer) > 0) && (atoll(buffer) > 0);
}

bool Sodaq_Ublox::getManufacturer(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("AT+CGMI", buffer, size);
}

bool Sodaq_Ublox::getModel(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("AT+CGMM", buffer, size);
}

bool Sodaq_Ublox::getFirmwareVersion(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("AT+CGMR", buffer, size);
}

bool Sodaq_Ublox::getFirmwareRevision(char* buffer, size_t size)
{
    if (buffer == NULL || size < 30 + 1) {
        return false;
    }

    return execCommand("ATI9", buffer, size);
}

/******************************************************************************
* RSSI and CSQ
*****************************************************************************/

bool Sodaq_Ublox::waitForSignalQuality(uint32_t timeout)
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

/**
 * Convert a CSQ value to RSSI
    The range is the following:
    0: -113 dBm or less
    1: -111 dBm
    2..30: from -109 to -53 dBm with 2 dBm steps
    31: -51 dBm or greater
    99: not known or not detectable or currently not available
*/
int8_t Sodaq_Ublox::convertCSQ2RSSI(uint8_t csq) const
{
    return -113 + 2 * csq;
}

/**
 * Convert a CESQ value to RSSI
    The range is the following:
    0: less than -110 dBm
    1..62: from -110 to -49 dBm with 1 dBm steps
    63: -48 dBm or greater
    99: not known or not detectable
*/
int8_t Sodaq_Ublox::convertCESQ2RSSI(uint8_t cesq) const
{
    return -109 + cesq;
}

uint8_t Sodaq_Ublox::convertRSSI2CSQ(int8_t rssi) const
{
    return (rssi + 113) / 2;
}

// Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
// Returns true if successful.
bool Sodaq_Ublox::getRSSIAndBER(int8_t* rssi, uint8_t* ber)
{
    /* FIXME
     * Please notice that the conversion of "ber" values are most likely
     * to be incorrect.
     */
    static char berValues[] = { 49, 43, 37, 25, 19, 13, 7, 0 }; // 3GPP TS 45.008 [20] subclause 8.2.4

    char buffer[128];
    println("AT+CSQ");
    if (readResponse(buffer, sizeof(buffer), "+CSQ: ") != GSMResponseOK) {
        return false;
    }

    int csq;
    int tmp_ber;

    if (sscanf(buffer, "%d,%d", &csq, &tmp_ber) != 2) {
        return false;
    }

    if (rssi) {
        *rssi = ((csq == 99) ? 0 : convertCSQ2RSSI(csq));
    }
    if (ber) {
        *ber  = ((tmp_ber == 99 ||
                 static_cast<size_t>(tmp_ber) >= sizeof(berValues)) ? 0 : berValues[tmp_ber]);
    }

    return true;
}

/**
 * Convert a RSRQ value to dBm
 * Reference Signal Received Quality (RSRQ):
 * • 0: -19 dB or less
 * • 1..33: from -19.5 dB to -3.5 dB with 0.5 dB steps
 * • 34: -3 dB or greater
 * • 255: not known or not detectable
*/
float Sodaq_Ublox::convertRSRQ2dBm(uint8_t rsrq) const
{
    if (rsrq <= 34) {
        // Using -20 not the -19 from the manual.
        return -20.0 + 0.5 * rsrq;
    }
    return 0.0;
}

/**
 * Convert a RSRP value to dBm
 * Reference Signal Received Power (RSRP):
 * • 0: -141 dBm or less
 * • 1..96: from -140 dBm to -45 dBm with 1 dBm steps
 * • 97: -44 dBm or greater
 * • 255: not known or not detectable
*/
float Sodaq_Ublox::convertRSRP2dBm(uint8_t rsrp) const
{
    if (rsrp <= 97) {
        return -141.0 + rsrp;
    }
    return 0.0;
}

/**
 * Get the Extended Signal Quality (AT+CESQ)
 *
 * \param[out] result Pointer to a struct for the result values
 *
 * AT+CESQ returns +CESQ: <rxlev>,<ber>,<rscp>,<ecn0>,<rsrq>,<rsrp>
 */
bool Sodaq_Ublox::getExtendedSignalQuality(ext_sig_qual_t * esq)
{
    char buffer[128];
    println("AT+CESQ");
    if (readResponse(buffer, sizeof(buffer), "+CESQ: ") != GSMResponseOK) {
        return false;
    }

    int rssi;
    int ber;
    int rscp;
    int ecn0;
    int rsrq;
    int rsrp;
    if (sscanf(buffer, "%d,%d,%d,%d,%d,%d",
               &rssi, &ber, &rscp, &ecn0, &rsrq, &rsrp) != 6) {
        return false;
    }

    if (esq) {
        memset(esq, 0, sizeof(*esq));
        esq->rssi = ((rssi == 99) ? 0 : convertCESQ2RSSI(rssi));
        esq->ber = ber;
        esq->rscp = rscp;
        esq->ecn0 = ecn0;
        esq->rsrq = convertRSRQ2dBm(rsrq);
        esq->rsrp = convertRSRP2dBm(rsrp);
    }

    return true;
}

int Sodaq_Ublox::socketCloseAll() {

    int closedCount = 0;

    for (uint8_t i = 0; i < SODAQ_UBLOX_SOCKET_COUNT; i++) {
        if (socketClose(i, false)) {
            closedCount++;
        }
    }

    // return the number of sockets we closed
    return closedCount;
}

size_t Sodaq_Ublox::socketGetPendingBytes(uint8_t socketID)
{
    return _socketPendingBytes[socketID];
}

bool Sodaq_Ublox::socketHasPendingBytes(uint8_t socketID)
{
    return socketGetPendingBytes(socketID) > 0;
}

bool Sodaq_Ublox::execCommand(const char* command, uint32_t timeout)
{
    println(command);

    return (readResponse(NULL, 0, NULL, timeout) == GSMResponseOK);
}

bool Sodaq_Ublox::execCommand(const String& command, uint32_t timeout)
{
    println(command);

    return (readResponse(NULL, 0, NULL, timeout) == GSMResponseOK);
}

bool Sodaq_Ublox::execCommand(const char* command, char* buffer, size_t size, uint32_t timeout)
{
    println(command);

    return (readResponse(buffer, size, NULL, timeout) == GSMResponseOK);
}

bool Sodaq_Ublox::execCommand(const String& command, char* buffer, size_t size, uint32_t timeout)
{
    println(command);

    return (readResponse(buffer, size, NULL, timeout) == GSMResponseOK);
}

/**
 * Wait for a prompt
 *
 * Most likely there is a <CR><LF> first.
 */
bool Sodaq_Ublox::waitForPrompt(char prompt, uint32_t timeout)
{
    uint32_t start_ts = millis();
    bool at_bol = true;
    bool retval = false;
    bool done_diag = false;

    do {
        int c = _modemUART->read();
        if (c < 0) {
            continue;
        }

        if (c == '\r') {
            at_bol = true;
        }
        else if (c == '\n') {
            at_bol = true;
        }
        else {
            if (at_bol) {
                debugPrint(">> ");
                at_bol = false;
                done_diag = true;
            }
            debugPrint((char)c);
        }
        if (c == prompt) {
            retval = true;
            break;
        }
    } while (!is_timedout(start_ts, timeout));

    if (done_diag && !at_bol) {
        debugPrintln();
    }
    return retval;
}

/**
 * 1. check echo
 * 2. check ok
 * 3. check error
 * 4. if response prefix is not empty, check response prefix, append if multiline
 * 5. check URC, if handled => continue
 * 6. if response prefix is empty, return the whole line return line buffer, append if multiline
*/
GSMResponseTypes Sodaq_Ublox::readResponse(char* outBuffer, size_t outMaxSize, const char* prefix, uint32_t timeout)
{
    bool usePrefix    = prefix != NULL && prefix[0] != 0;
    bool useOutBuffer = outBuffer != NULL && outMaxSize > 0;

    uint32_t from = millis();

    size_t outSize = 0;

    if (outBuffer) {
        outBuffer[0] = 0;
    }

    //debugPrintln(String("[readResponse] timeout: ") + timeout);
    while (!is_timedout(from, timeout)) {
        int count = readLn(250);        // 250ms, how many bytes at which baudrate?
        sodaq_wdt_reset();

        if (count <= 0) {
            continue;
        }

        debugPrint("<< ");
        debugPrintln(getInputBuffer());

        if (startsWith("AT", getInputBuffer())) {
            continue; // skip echoed back command
        }

        if (startsWith("OK", getInputBuffer())) {
            return GSMResponseOK;
        }

        if (startsWith("ERROR", getInputBuffer()) ||
            startsWith("+CME ERROR:", getInputBuffer()) ||
            startsWith("+CMS ERROR:", getInputBuffer())) {
            return GSMResponseError;
        }

        bool hasPrefix = usePrefix && useOutBuffer && startsWith(prefix, getInputBuffer());

        if (!hasPrefix && checkURC(getInputBuffer())) {
            continue;
        }

        if (hasPrefix || (!usePrefix && useOutBuffer)) {
            /* Notice that the minus one is to guarantee that there is space to
             * add a NUL byte at the end.
             */
            if (outSize > 0 && outSize < outMaxSize - 1) {
                outBuffer[outSize++] = '\n';
            }

            if (outSize < outMaxSize - 1) {
                const char* inBuffer = getInputBuffer();
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

    debugPrintln("[readResponse] timed out");

    return GSMResponseTimeout;
}

// Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
int Sodaq_Ublox::timedRead(uint32_t timeout) const
{
    uint32_t _startMillis = millis();

    do {
        int c = _modemUART->read();

        if (c >= 0) {
            return c;
        }
    } while (!is_timedout(_startMillis, timeout));

    return -1; // -1 indicates timeout
}

// Fills the given "buffer" with characters read from the modem stream up to "length"
// maximum characters and until the "terminator" character is found or a character read
// times out (whichever happens first).
// The buffer does not contain the "terminator" character or a null terminator explicitly.
// Returns the number of characters written to the buffer, not including null terminator.
size_t Sodaq_Ublox::readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout)
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
size_t Sodaq_Ublox::readBytes(uint8_t* buffer, size_t length, uint32_t timeout)
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

// Reads a line (up to the terminator) from the modem stream into the "buffer".
// The buffer is terminated with null.
// Returns the number of bytes read, not including the null terminator.
size_t Sodaq_Ublox::readLn(char* buffer, size_t size, uint32_t timeout)
{
    // Use size-1 to leave room for a string terminator
    size_t len = readBytesUntil(SODAQ_UBLOX_TERMINATOR[SODAQ_UBLOX_TERMINATOR_LEN - 1], buffer, size - 1, timeout);

    // check if the terminator is more than 1 characters, then check if the first character of it exists
    // in the calculated position and terminate the string there
    if ((SODAQ_UBLOX_TERMINATOR_LEN > 1) &&
        (buffer[len - (SODAQ_UBLOX_TERMINATOR_LEN - 1)] == SODAQ_UBLOX_TERMINATOR[0])) {
        len -= SODAQ_UBLOX_TERMINATOR_LEN - 1;
    }

    // terminate string, there should always be room for it (see size-1 above)
    buffer[len] = '\0';

    return len;
}

void Sodaq_Ublox::writeProlog()
{
    if (!_appendCommand) {
        debugPrint(">> ");
        _appendCommand = true;
    }
}

// Write a byte, as binary data
size_t Sodaq_Ublox::writeByte(uint8_t value)
{
    return _modemUART->write(value);
}

size_t Sodaq_Ublox::print(const String& buffer)
{
    writeProlog();
    debugPrint(buffer);

    return _modemUART->print(buffer);
}

size_t Sodaq_Ublox::print(const char buffer[])
{
    writeProlog();
    debugPrint(buffer);

    return _modemUART->print(buffer);
}

size_t Sodaq_Ublox::print(char value)
{
    writeProlog();
    debugPrint(value);

    return _modemUART->print(value);
};

size_t Sodaq_Ublox::print(unsigned char value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemUART->print(value, base);
};

size_t Sodaq_Ublox::print(int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemUART->print(value, base);
};

size_t Sodaq_Ublox::print(unsigned int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemUART->print(value, base);
};

size_t Sodaq_Ublox::print(long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemUART->print(value, base);
};

size_t Sodaq_Ublox::print(unsigned long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemUART->print(value, base);
};

size_t Sodaq_Ublox::println(const __FlashStringHelper *ifsh)
{
    return print(ifsh) + println();
}

size_t Sodaq_Ublox::println(const String &s)
{
    return print(s) + println();
}

size_t Sodaq_Ublox::println(const char c[])
{
    return print(c) + println();
}

size_t Sodaq_Ublox::println(char c)
{
    return print(c) + println();
}

size_t Sodaq_Ublox::println(unsigned char b, int base)
{
    return print(b, base) + println();
}

size_t Sodaq_Ublox::println(int num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_Ublox::println(unsigned int num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_Ublox::println(long num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_Ublox::println(unsigned long num, int base)
{
    return print(num, base) + println();
}

size_t Sodaq_Ublox::println(double num, int digits)
{
    writeProlog();
    debugPrint(num, digits);

    return _modemUART->println(num, digits);
}

size_t Sodaq_Ublox::println(const Printable& x)
{
    return print(x) + println();
}

size_t Sodaq_Ublox::println()
{
    debugPrintln();
    size_t i = print('\r');
    _appendCommand = false;
    return i;
}

void Sodaq_Ublox::dbprintln()
{
    debugPrintln();
    _appendCommand = false;
}

/******************************************************************************
* Utils
*****************************************************************************/

uint32_t Sodaq_Ublox::convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec)
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

bool Sodaq_Ublox::startsWith(const char* pre, const char* str)
{
    return (strncmp(pre, str, strlen(pre)) == 0);
}

bool Sodaq_Ublox::isValidIPv4(const char* str)
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
