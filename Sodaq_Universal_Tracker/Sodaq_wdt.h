/*
Copyright (c) 2016-19, SODAQ
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


#ifndef SODAQ_WDT_H_
#define SODAQ_WDT_H_

#ifdef ARDUINO_ARCH_AVR

#include <avr/wdt.h>

#endif

/*
  To avoid naming conflicts with 'avr/wdt.h', the 'sodaq_' prefix has
  been attached to these methods. e.g. 'sodaq_wdt_reset()'.

  There is an enumeration, called 'wdt_period', of the possible interrupt 
  periods ranging from 15ms to 8s. The SAMD also supports an 8ms period, but 
  this has been omitted as the AVR platform doesn't. Some of the shorter 
  periods are an approximate equivalent e.g.AVR = 120ms SAMD = 125ms.

  On the SAMD platform this library uses 'Normal Mode' with the 'Early Warning' 
  interrupt enabled. The 'Early Warning' interrupt is set to the specified 
  period, and the board reset to twice that. So if a period of 1s is specified 
  the interrupt will fire after 1s and the board will reset 1s afterwards 
  (if 'sodaq_wdt_reset()' is not called before).

  Because the board reset period is set to twice that of the interrupt period, 
  the 16s period on the SAMD board is used when an 8s period is specified.

  A safe delay method, 'sodaq_wdt_safe_delay(ms)', has been added. This method 
  will block and only return after the specified number of milliseconds have 
  elapsed. It calls 'delay(ms)' in 10ms increments, and calls 
  'sodaq_wdt_reset()' between.
*/

// Approximate periods supported by both platforms
// The SAMD also supports 8ms and 16s.
enum wdt_period : uint8_t 
{
#ifdef ARDUINO_ARCH_AVR

  // See avr/wdt.h
  WDT_PERIOD_1DIV64  = WDTO_15MS,  // 15ms   = ~1/64s
  WDT_PERIOD_1DIV32  = WDTO_30MS,  // 30ms   = ~1/32s
  WDT_PERIOD_1DIV16  = WDTO_60MS,  // 60ms   = ~1/16s
  WDT_PERIOD_1DIV8   = WDTO_120MS, // 120ms  = ~1/8s
  WDT_PERIOD_1DIV4   = WDTO_250MS, // 250ms  = 1/4s
  WDT_PERIOD_1DIV2   = WDTO_500MS, // 500ms  = 1/2s
  WDT_PERIOD_1X      = WDTO_1S,    // 1000ms = 1s
  WDT_PERIOD_2X      = WDTO_2S,    // 2000ms = 2s
  WDT_PERIOD_4X      = WDTO_4S,    // 4000ms = 4s
  WDT_PERIOD_8X      = WDTO_8S     // 8000ms = 8s
  
#elif ARDUINO_ARCH_SAMD  

  // See Arduino15/packages/arduino/tools/CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/component/wdt.h
  // It is easier to use numeric values as there are two
  // sets of macros one for the reset WDT_CONFIG_WINDOW()
  // and the other for the early warning period WDT_EWCTRL_EWOFFSET().
  // Clock is running at 1024hz
  WDT_PERIOD_1DIV64 = 1,   // 16 cycles   = 1/64s
  WDT_PERIOD_1DIV32 = 2,   // 32 cycles   = 1/32s
  WDT_PERIOD_1DIV16 = 3,   // 64 cycles   = 1/16s
  WDT_PERIOD_1DIV8  = 4,   // 128 cycles  = 1/8s
  WDT_PERIOD_1DIV4  = 5,   // 256 cycles  = 1/4s
  WDT_PERIOD_1DIV2  = 6,   // 512 cycles  = 1/2s
  WDT_PERIOD_1X     = 7,   // 1024 cycles = 1s
  WDT_PERIOD_2X     = 8,   // 2048 cycles = 2s
  WDT_PERIOD_4X     = 9,   // 4096 cycles = 4s
  WDT_PERIOD_8X     = 10   // 8192 cycles = 8s
  
#endif
};

void sodaq_wdt_enable(wdt_period period = WDT_PERIOD_1X);

void sodaq_wdt_disable();

void sodaq_wdt_reset();

void sodaq_wdt_safe_delay(uint32_t ms);

extern volatile bool sodaq_wdt_flag;

#endif /* SODAQ_WDT_H_ */
