/**************************************************************************/
/*!
    @file     RotaryEncoder.h
    @author   Jason Scheunemann <jason.scheunemann@gmail.com>

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Jason Scheunemann
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#ifndef RotaryEncoder_h
#define RotaryEncoder_h

#include <Arduino.h>
#include <limits.h>

#define DEBOUNCE_DELAY 35

#define PIN_A 2
#define PIN_B 3

class RotaryEncoder;
typedef void (*rotaryEncoderEventHandler)(RotaryEncoder&);

class RotaryEncoder {
  public:
    RotaryEncoder::RotaryEncoder(byte pinAInterrupt, byte pinBInterrupt);
    void RotaryEncoder::begin();
    int RotaryEncoder::read();
    void RotaryEncoder::incrementHandler(rotaryEncoderEventHandler handler);
    void RotaryEncoder::decrementHandler(rotaryEncoderEventHandler handler);
  private:
    void RotaryEncoder::pinAChange();
    void RotaryEncoder::pinBChange();
    rotaryEncoderEventHandler cb_onIncrement;
    rotaryEncoderEventHandler cb_onDecrement;
    volatile unsigned int encoderPos = 0;
    unsigned int pos = 0;
    volatile byte pinAState = LOW;
    volatile byte pinBState = LOW;
    long lastUpdate;
    byte pinAInterrupt;
    byte pinBInterrupt;
};

#endif
