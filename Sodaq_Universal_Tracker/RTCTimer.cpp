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


/*
 * The RTCTimer can be used to implement a simple scheduler.
 * It was inspired by the Arduino Timer library by Simon Monk.
 */

#include "RTCTimer.h"
//#include "Diag.h"

#if 0
RTCEvent::RTCEvent()
{
  _eventType = RTCEvent_None;
  _lastEventTime = 0;
  _period = 0;
  _callback = 0;
  _count = 0;
  _repeatCount = 0;
}

RTCTimer::RTCTimer()
{
}
#endif

bool RTCEvent::update(uint32_t now)
{
  bool doneEvent = false;
  if ((int32_t)(now - (_lastEventTime + _period)) >= 0) {
    //DIAGPRINT(F("RTCEvent::update ")); DIAGPRINTLN(_lastEventTime);
    if (_lastEventTime == 0) {
      // In case this wasn't initialized properly
      _lastEventTime = now;
    } else {
      while ((int32_t)(now - (_lastEventTime + _period)) >= 0) {
        // Skip all the events that are in the past
        _lastEventTime += _period;
      }
    }
    switch (_eventType) {
    case RTCEvent_Every:
      (*_callback)(now);
      break;
    default:
      break;
    }
    if (_repeatCount > 0 && ++_count >= _repeatCount) {
      // Done. Free the event.
      _eventType = RTCEvent_None;
    }
    doneEvent = true;
  }
  return doneEvent;
}

int8_t RTCTimer::every(uint32_t period, void (*callback)(uint32_t now),
    int repeatCount)
{
  int8_t i = findFreeEventIndex();
  if (i == -1)
    return -1;

  _events[i]._eventType = RTCEvent::RTCEvent_Every;
  _events[i]._period = period;
  _events[i]._repeatCount = repeatCount;
  _events[i]._callback = callback;
  _events[i]._lastEventTime = _now ? _now() : 0;
  _events[i]._count = 0;

  return i;
}

/*
 * Reset the "last event" time of all events to the new value.
 *
 * This function should be called once (e.g. in setup() after creating
 * all the ...
 */
void RTCTimer::resetAll(uint32_t now)
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType != RTCEvent::RTCEvent_None) {
      _events[i]._lastEventTime = now;
    }
  }
}

void RTCTimer::clearAllEvents()
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    _events[i]._eventType = RTCEvent::RTCEvent_None;
  }
}

void RTCTimer::adjust(uint32_t old, uint32_t now)
{
  if (old == 0) {
    resetAll(now);
    return;
  }
  int32_t diff = now - old;
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType != RTCEvent::RTCEvent_None) {
      if (_events[i]._lastEventTime == 0) {
        _events[i]._lastEventTime = now;
      } else {
        _events[i]._lastEventTime += diff;
      }
    }
  }
}

void RTCTimer::update()
{
  uint32_t now = _now ? _now() : 0;
  if (now) {
    update(now);
  }
}

void RTCTimer::update(uint32_t now)
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType != RTCEvent::RTCEvent_None) {
      if (_events[i].update(now)) {
        if (_noMultipleEvents) {
          // Only one event action per update.
          break;
        }
      }
    }
  }
}

int8_t RTCTimer::findFreeEventIndex()
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType == RTCEvent::RTCEvent_None) {
      return i;
    }
  }
  return -1;
}
