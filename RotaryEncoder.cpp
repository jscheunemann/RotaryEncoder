/**************************************************************************/
/*!
    @file     RotaryEncoder.cpp
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

#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(byte pinAInterrupt, byte pinBInterrupt) {
  this->pinAInterrupt = pinAInterrupt;
  this->pinBInterrupt = pinBInterrupt;
}

void RotaryEncoder::begin() {
  pinMode(this->pinAInterrupt, INPUT);
  pinMode(this->pinBInterrupt, INPUT);

  digitalWrite(this->pinAInterrupt, HIGH);
  digitalWrite(this->pinBInterrupt, HIGH);

  // attachInterrupt(digitalPinToInterrupt(this->pinAInterrupt), this->pinAChange, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(this->pinBInterrupt), this->pinBChange, CHANGE);

  lastUpdate = millis();
  encoderPos = INT_MAX >> 1;
}

int RotaryEncoder::read() {
  if (millis() - lastUpdate > DEBOUNCE_DELAY) {
    if (encoderPos != INT_MAX >> 1) {
      if (encoderPos > INT_MAX >> 1) {
        pos++;
        cb_onIncrement(*this);
      }
      else if (encoderPos < INT_MAX >> 1) {
        pos--;
        cb_onDecrement(*this);
      }
    }

    lastUpdate = millis();
    encoderPos = INT_MAX >> 1;
  }

  return pos;
}

void RotaryEncoder::incrementHandler(rotaryEncoderEventHandler handler) {
  this->cb_onIncrement = handler;
}

void RotaryEncoder::decrementHandler(rotaryEncoderEventHandler handler) {
  this->cb_onDecrement = handler;
}

void RotaryEncoder::pinAChange() {
  if ((this->pinAState = digitalRead(this->pinAInterrupt) == HIGH) ^ this->pinBState) {
    this->encoderPos++;
  }
}

void RotaryEncoder::pinBChange() {
  if ((this->pinBState = digitalRead(this->pinBInterrupt) == HIGH) ^ this->pinAState) {
    this->encoderPos--;
  }
}
