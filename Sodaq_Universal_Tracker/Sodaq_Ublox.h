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

#ifndef _SODAQ_UBLOX_H
#define _SODAQ_UBLOX_H

#include <stdint.h>

#include <Arduino.h>

#define SODAQ_UBLOX_DEFAULT_RESPONSE_TIMEOUT    5000
#define SODAQ_UBLOX_DEFAULT_SOCKET_TIMEOUT      15000

#define SODAQ_UBLOX_SOCKET_COUNT        7

enum GSMResponseTypes {
    GSMResponseNotFound = 0,
    GSMResponseOK = 1,
    GSMResponseError = 2,
    GSMResponseSocketPrompt = 3,
    GSMResponseFilePrompt = 4,
    GSMResponseTimeout = 5,
    GSMResponseEmpty = 6,
};

enum Protocols {
    TCP = 0,
    UDP
};

class Sodaq_OnOffBee
{
public:
    virtual ~Sodaq_OnOffBee() {}
    virtual void on()   = 0;
    virtual void off()  = 0;
    virtual bool isOn() = 0;
};

/**
 * Extended Signal Quality
 *
 * This struct is used as a collection of the values returned by the AT+CESQ
 * command.
 */
typedef struct
{
    int16_t rssi;       //< Received Signal Strength Indication (RSSI) (dBm)
    uint8_t ber;        //< Bit Error Rate (BER)
    uint8_t rscp;       //< Received Signal Code Power (RSCP)
    uint8_t ecn0;       //< Ratio of received energy per PN chip to the
                        //  total received power spectral density
    float rsrq;        //< Reference Signal Received Quality (RSRQ)
    float rsrp;        //< Reference Signal Received Power (RSRP)
    float snr;         //< Signal to noise ratio
} ext_sig_qual_t;

class Sodaq_Ublox
{
public:
    Sodaq_Ublox();
    virtual ~Sodaq_Ublox();

    // Turns the modem on/off and returns true if successful.
    virtual bool on() = 0;
    virtual bool off() = 0;

    // Sets the size of the input buffer.
    // Needs to be called before init().
    void setInputBufferSize(size_t value) { _inputBufferSize = value; };

    /******************************************************************************
     * Connect / Disconnect
     *****************************************************************************/

    virtual bool connect() = 0;
    void    setCid(uint8_t cid) { _cid = cid; }
    void    setConnectTimeout(uint32_t t) { _connect_timeout = t; }

    void    setApn(const char* apn) { _apn = apn; }

    /******************************************************************************
    * Sockets
    *****************************************************************************/

    virtual int socketCreate(uint16_t localPort = 0, Protocols protocol = UDP) = 0;
    virtual bool socketClose(uint8_t socketID, bool async = false) = 0;
    int    socketCloseAll();

    // UDP only
    virtual size_t socketSend(uint8_t socketID,
            const char* remoteHost, const uint16_t remotePort,
            const uint8_t* buffer, size_t size) = 0;
    virtual bool   socketWaitForReceive(uint8_t socketID,
            uint32_t timeout = SODAQ_UBLOX_DEFAULT_SOCKET_TIMEOUT) = 0;
    virtual size_t socketReceive(uint8_t socketID, uint8_t* buffer, size_t length) = 0;

    size_t socketGetPendingBytes(uint8_t socketID);
    bool   socketHasPendingBytes(uint8_t socketID);

    // Timeouts for socket functions
    void   setSocketWriteTimeout(uint32_t t) { _socket_write_timeout = t; }

    /******************************************************************************
     * Generic Info
     *****************************************************************************/

    bool getCCID(char* buffer, size_t size);
    bool getIMSI(char* buffer, size_t size);
    bool getManufacturer(char* buffer, size_t size);
    bool getModel(char* buffer, size_t size);
    bool getFirmwareVersion(char* buffer, size_t size);
    bool getFirmwareRevision(char* buffer, size_t size);
    virtual bool getIMEI(char* buffer, size_t size) = 0;
    virtual bool getEpoch(uint32_t* epoch) = 0;
    virtual bool getOperatorInfo(uint16_t* mcc, uint16_t* mnc) = 0;
    virtual bool getOperatorInfoNumber(char* buffer, size_t size) = 0;
    virtual bool getOperatorInfoString(char* buffer, size_t size) = 0;
    virtual bool getCellInfo(uint16_t* tac, uint32_t* cid, uint16_t* urat) = 0;

    /******************************************************************************
     * RSSI and CSQ
     *****************************************************************************/

    int8_t  convertCSQ2RSSI(uint8_t csq) const;
    int8_t  convertCESQ2RSSI(uint8_t cesq) const;
    uint8_t convertRSSI2CSQ(int8_t rssi) const;

    float  convertRSRQ2dBm(uint8_t rsrq) const;
    float  convertRSRP2dBm(uint8_t rsrp) const;

    uint8_t getCSQtime()  const { return _CSQtime; }
    int8_t  getLastRSSI() const { return _lastRSSI; }
    int8_t  getMinRSSI()  const { return _minRSSI; }

    // Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
    // Returns true if successful.
    bool    getRSSIAndBER(int8_t* rssi, uint8_t* ber);

    void    setMinCSQ(int csq) { _minRSSI = convertCSQ2RSSI(csq); }
    void    setMinRSSI(int rssi) { _minRSSI = rssi; }

    virtual bool getExtendedSignalQuality(ext_sig_qual_t * esq);

    /* Generic function to execute whatever AT command
     */
    bool    execCommand(const char* command, uint32_t timeout = SODAQ_UBLOX_DEFAULT_RESPONSE_TIMEOUT);
    bool    execCommand(const String& command, uint32_t timeout = SODAQ_UBLOX_DEFAULT_RESPONSE_TIMEOUT);
    bool    execCommand(const char* command, char* buffer, size_t size,
                        uint32_t timeout = SODAQ_UBLOX_DEFAULT_RESPONSE_TIMEOUT);
    bool    execCommand(const String& command, char* buffer, size_t size,
                        uint32_t timeout = SODAQ_UBLOX_DEFAULT_RESPONSE_TIMEOUT);

    // Sets the optional "Diagnostics and Debug" print.
    void setDiag(Print &print) { _diagPrint = &print; }
    void setDiag(Print *print) { _diagPrint = print; }

protected:
    /***********************************************************/
    /* UART */

    void initUART(Uart& uart, uint32_t baud) { _modemUART = &uart; _baudRate = baud; }

    /***********************************************************/
    /* ON-OFF */

    // Sets the onoff instance
    void setOnOff(Sodaq_OnOffBee * onoff) { _onoff = onoff; }

    // Returns true if the modem is on.
    bool isOn() const;

    // Returns true if the modem replies to "AT" commands without timing out.
    bool isAlive(size_t retry_count = 0);

#define SODAQ_UBLOX_DEFAULT_CSQ_TIMEOUT         (5L * 60L * 1000)
    bool   waitForSignalQuality(uint32_t timeout = SODAQ_UBLOX_DEFAULT_CSQ_TIMEOUT);

    /***********************************************************/
    /* Serial In/Out to the modem via the UART */

    // Initializes the input buffer and makes sure it is only initialized once.
    // Safe to call multiple times.
    void initBuffer();

    const char* getInputBuffer() const { return _inputBuffer; }

    bool waitForPrompt(char prompt, uint32_t timeout);

    GSMResponseTypes readResponse(char* outBuffer = NULL, size_t outMaxSize = 0, const char* prefix = NULL,
                                  uint32_t timeout = SODAQ_UBLOX_DEFAULT_RESPONSE_TIMEOUT);
    virtual bool checkURC(const char* buffer) = 0;

    // Returns a character from the modem UART if read within _timeout ms or -1 otherwise.
    int timedRead(uint32_t timeout = 1000) const;

    // Fills the given "buffer" with characters read from the modem UART up to "length"
    // maximum characters and until the "terminator" character is found or a character read
    // times out (whichever happens first).
    // The buffer does not contain the "terminator" character or a null terminator explicitly.
    // Returns the number of characters written to the buffer, not including null terminator.
    size_t readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout = 1000);

    // Fills the given "buffer" with up to "length" characters read from the modem UART.
    // It stops when a character read times out or "length" characters have been read.
    // Returns the number of characters written to the buffer.
    size_t readBytes(uint8_t* buffer, size_t length, uint32_t timeout = 1000);

    // Reads a line from the modem UART into the input buffer.
    // Returns the number of bytes read.
    size_t readLn(uint32_t timeout = 1000) { return readLn(_inputBuffer, _inputBufferSize, timeout); };

    // Reads a line from the modem UART into the "buffer". The line terminator is not
    // written into the buffer. The buffer is terminated with null.
    // Returns the number of bytes read, not including the null terminator.
    size_t readLn(char* buffer, size_t size, uint32_t timeout = 1000);

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
    void dbprintln();

    /******************************************************************************
    * Utils
    *****************************************************************************/

    static uint32_t convertDatetimeToEpoch(int y, int m, int d, int h, int min, int sec);
    static bool startsWith(const char* pre, const char* str);
    static bool isValidIPv4(const char* str);

protected:
    // Determine the current baudrate
    uint32_t determineBaudRate(uint32_t current);
    virtual uint32_t getNthValidBaudRate(size_t nth) = 0;
    bool isValidSocketID(int id) { return id >= 0 && id < SODAQ_UBLOX_SOCKET_COUNT; }

    // The (optional) stream to show debug information.
    Print*      _diagPrint;

    // The on-off pin power controller object.
    Sodaq_OnOffBee* _onoff;

    // The UART that communicates with the device.
    Uart*       _modemUART;

    // The requested baudrate
    uint32_t    _baudRate;

    // Keep track when connect started. Use this to record various status changes.
    uint32_t    _startOn;
    const char* _apn;
    uint8_t     _cid;

    uint32_t    _connect_timeout;
    uint32_t    _disconnect_timeout;

    bool        _socketClosed[SODAQ_UBLOX_SOCKET_COUNT];
    size_t      _socketPendingBytes[SODAQ_UBLOX_SOCKET_COUNT];

    uint32_t    _socket_close_timeout;
    uint32_t    _socket_connect_timeout;
    uint32_t    _socket_write_timeout;

private:
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
};

#endif /* _SODAQ_UBLOX_H */
