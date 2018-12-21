/*
Copyright (c) 2014-18, Kees Bakker and SODAQ
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


#include <stdint.h>

#ifndef RTCTIMER_H_
#define RTCTIMER_H_

#define MAX_NUMBER_OF_RTCEVENTS (10)

class RTCTimer;
class RTCEvent
{
  friend class RTCTimer;
public:
  enum RTCEventType {
    RTCEvent_None = 0,
    RTCEvent_Every,
  };
  //RTCEvent();

  bool update(uint32_t now);

protected:
  enum RTCEventType _eventType;
  uint32_t _lastEventTime;
  uint32_t _period;
  int _count;
  int _repeatCount;
  void (*_callback)(uint32_t now);
};

class RTCTimer
{
public:
  //RTCTimer();

  int8_t every(uint32_t period, void (*callback)(uint32_t ts), int repeatCount=-1);

  void resetAll(uint32_t now);
  void clearAllEvents();
  void setNowCallback(uint32_t (*now)()) { _now = now; }
  void adjust(uint32_t old, uint32_t now);
  void update();
  void update(uint32_t now);
  void allowMultipleEvents(bool allow=true) { _noMultipleEvents = !allow; }

protected:
  int8_t        findFreeEventIndex();
  uint32_t      (*_now)();
  RTCEvent      _events[MAX_NUMBER_OF_RTCEVENTS];
  bool          _noMultipleEvents;
};

#endif /* RTCTIMER_H_ */
