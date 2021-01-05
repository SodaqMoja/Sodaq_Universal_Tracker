# SODAQ - Universal Tracker

Note: to be able to compile this application you need to add the right board file to your Arduino IDE.

Go to File, Preferences and set the following URL for the additional board files:

http://downloads.sodaq.net/package_sodaq_samd_index.json

You should install (or update to) version 1.6.19 or higher.

## Configuration Menu

After compiling the source code and uploading it to the board you will be able to configure the board through a menu.

Just open the Arduino Serial Monitor (at 115200 baud, with line endings set to NL/CR) and you will get this menu:

[In the case of SARA AFF or SFF]

```Arduino
** SODAQ - Universal Tracker - 1.0.4 **

 -> CPU reset by Software [64]


Commands:
  Show IMEI                                  (SI):
  Show CCID                                  (SC):
  Show Module version                        (SV):
  Commit Settings                            (CS):

Settings:

GPS
  Fix Interval (min)         (fi=): 15
  Alt. Fix Interval (min)    (afi=): 0
  Alt. Fix From (HH)         (affh=): 0
  Alt. Fix From (MM)         (affm=): 0
  Alt. Fix To (HH)           (afth=): 0
  Alt. Fix To (MM)           (aftm=): 0
  GPS Fix Timeout (sec)      (gft=): 120
  Minimum sat count          (sat=): 4
  Num Coords to Upload       (num=): 1

On-the-move Functionality
  Acceleration% (100% = 8g)  (acc=): 0
  Acceleration Duration      (acd=): 0
  Fix Interval (min)         (acf=): 1
  Timeout (min)              (act=): 10

Cellular
  Network Type (N2xx NB-IoT = 2, N3xx NB-IoT = 3, R4xx NB-IoT = 4, R4xx LTE-M = 5, R412 2G = 6, 2G/3G = 7) (ntype=): 0
  All Things Talk Token      (att=):
  APN                        (apn=): nb.inetd.gdsp
  Force Operator             (opr=): 0
  CID                        (cid=): 1
  MNO Profile                (mno=): 1
  APN user                   (apnu=):
  APN password               (apnp=):
  Band Info                  (bnd=): 524416
  Bands (N3xx NB-IoT only)   (bns=): 1800,850,900,800,700
  Target IP                  (ip=): 0.0.0.0
  Target port                (prt=): 1
  Response Timeout           (rxto=): 15

Misc
  Status LED (OFF=0 / ON=1)  (led=): 0
  Debug (OFF=0 / ON=1)       (dbg=): 0
Enter command:
```

[In the case of SODAQ ONE]

```Arduino
** SODAQ - Universal Tracker - 1.0.4 **
LoRa HWEUI: 0004A30B001FB4C1
 -> CPU reset by Software [64]


Commands:
  Reset DevAddr / DevEUI to the Hardware EUI (EUI):
  Reset LoRa                                 (RESET):
  Commit Settings                            (CS):

Settings:

GPS
  Fix Interval (min)         (fi=): 15
  Alt. Fix Interval (min)    (afi=): 0
  Alt. Fix From (HH)         (affh=): 0
  Alt. Fix From (MM)         (affm=): 0
  Alt. Fix To (HH)           (afth=): 0
  Alt. Fix To (MM)           (aftm=): 0
  GPS Fix Timeout (sec)      (gft=): 120
  Minimum sat count          (sat=): 4
  Num Coords to Upload       (num=): 1

On-the-move Functionality
  Acceleration% (100% = 8g)  (acc=): 0
  Acceleration Duration      (acd=): 0
  Fix Interval (min)         (acf=): 1
  Timeout (min)              (act=): 10

LoRa
  OTAA Mode (OFF=0 / ON=1)   (otaa=): 0
  Retry conn. (OFF=0 / ON=1) (retry=): 1
  ADR (OFF=0 / ON=1)         (adr=): 1
  ACK (OFF=0 / ON=1)         (ack=): 0
  Spreading Factor           (sf=): 7
  Output Index               (pwr=): 1
  Lora Port                  (lprt=): 1
  DevAddr / DevEUI           (dev=): 0004A30B001FB4C1
  AppSKey / AppEUI           (app=): 00000000000000000000000000000000
  NWSKey / AppKey            (key=): 00000000000000000000000000000000
  Repeat Count               (rep=): 0

Misc
  Cayenne LPP (OFF=0 / ON=1) (cay=): 0
  Status LED (OFF=0 / ON=1)  (led=): 0
  Debug (OFF=0 / ON=1)       (dbg=): 0
Enter command:
```

Entering commands is just a matter of typing the command as given in brackets with the right value. For example:

fi=5

Will set the time between the GPS fixes to 5 minutes.
Setting fi=0 will disable the default fix interval.

### Sleep

After the startup the device by default willt be in deep sleep mode. In sleep it uses less than 50 uA. Using the RTC Timers it will wake up at the set intervals.

### Timers and Watchdog

The application is based on the RTCTimer library. At startup the application tries to obtain a GPS fix until timeout. If no fix can be obtained initially the location will be set to 0,0. Once the first fix is obtained the RTC will be set and that fix location will be kept until the next proper fix.

There is a system watchdog running in case the application hangs it will be restarted by the watchdog. (See library Sodaq\_wdt)

### GPS fixes

As seen in the configuration menu we allow for two different schedules for GPS fixes based on UTC time. The default could for instance be that we want a GPS fix every 30 minutes during the night, but during the day we want a fix every 5 minutes. In that case we configure the default to be 30 minutes, but the optional fix timer to be 5 minutes from 06:00 UTC to 18:00 UTC.

The RTC library allows for two of such timers. In case the second Fix interval is set to 0 the second timer is simply ignored.

After a GPS fix is obtained with at least 'Minimum sat count' satellites (or the timeout reached) a LoRa packet will be sent. In case of timeout without obtaining enough satellites,
the best GPS fix found (if any) is used.

For redundancy we could configure a repeat count. The value of the repeat count tells us to send the Lora frame an additional number of times (default 0) for redundancy.

The Lora frame contains the following data. The minimum frame size is 21 bytes, the maximum frame size 51 bytes, depending on the number of coordinates we have configured to be sent.

### Band settings

From 1.0.4 there is support for multiband selection.

Bands 8 and 20 can be set for the N2XX by bnd=8,20
Band Info                (bnd=): 8,20

Bands 8 and 20 can be set for the R4XX by bnd=524416
524416 is the decimal presentation of the 8th and 20th bit. 10000000000010000000
Band Info                (bnd=): 524416

### LoRa Connection

The LoRa communication only starts if the keys are not 0 (0 is the default)
If 'Retry conn.' is on, then in case the connection to the network is not successful (useful for OTAA), the application will retry to connect the next time there is a pending transmission.

### Temperature

The on-board accelerometer provides temperature delta with 1 degree Celsius resolution. It is not factory calibrated so the offset needs to be set in the application (command "temp" or it can be hardcoded in the code) for each board.

### On-the-move Functionality

The firmware supports, except for the default and alternative fix intervals, a third fix interval that is dependent to movement: if the acceleration on any axis goes over (or below in the case of a negative axis) the acceleration set in Acceleration% parameter for over the set duration, the on-the-move fix interval becomes active until "Timeout" minutes have passed since the last movement detected.

### Message Frame content

Note: all values are little endian.

| Description | Length |
| --- | --- |
| Epoch Timestamp | uint32 (4) |
| Battery voltage (between 3 and 4.5 V) | uint8 (1) |
| Board Temperature (degrees celsius) | int8 (1) |
| Lat | int32 (4) |
| Long | int32 (4) |
| Altitude (MSL in meters) | int16 (2) |
| Speed (SOG \* 100 km/h) | uint16 (2) |
| Course (COG) | uint8 (1) |
| Number of satellites | uint8 (1) |
| Time to fix (seconds, FF = no fix, in that case the above position is the last known) | uint8 (1) |
| Plus 0 - 3 of the following 10 bytes: |   |
| Previous fix (seconds ago, FFFF means longer than) | uint16 (2) |
| Lat | long (4) |
| Long | long(4) |

### Remote (over the air) re-configuration

You can send the following configuration parameters back to the device (as part as the LoRaWAN class A communication protocol)

| Description | Length |
| --- | --- |
| Fix Interval  default | uint16 (2) |
| Fix Interval alternate | uint16 (2) |
| From (EPOCH) | long (4) |
| To (EPOCH) | long (4) |
| GPS fix timeout in seconds | uint16 (2) |

## License

Copyright (c) 2018-2020, SODAQ
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
