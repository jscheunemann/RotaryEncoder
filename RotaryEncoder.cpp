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

  attachInterrupt(digitalPinToInterrupt(PIN_A), int0ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), int1ISR, CHANGE);

  this->instance_ = this;
  this->pos = 0;
  this->lastPos = 0;
  this->pinAState = LOW;
  this->pinBState = LOW;

  this->lastUpdate = millis();
  this->encoderPos = INT_MAX >> 1;

}

long RotaryEncoder::read() {
  if (millis() - this->lastUpdate > DEBOUNCE_DELAY) {
    if (this->encoderPos != INT_MAX >> 1) {
      if (this->encoderPos > INT_MAX >> 1) {
        if (++this->pos / 2 > this->lastPos) {
          cb_onIncrement(*this);
        }
      }
      else if (this->encoderPos < INT_MAX >> 1) {
        if (--this->pos / 2 < this->lastPos) {
          cb_onDecrement(*this);
        }
      }
    }

    this->lastPos = this->pos / 2;
    this->lastUpdate = millis();
    this->encoderPos = INT_MAX >> 1;
  }

  return this->pos / 2;
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

void RotaryEncoder::int0ISR() {
  RotaryEncoder::instance_->pinAChange();
}

void RotaryEncoder::int1ISR() {
  RotaryEncoder::instance_->pinBChange();
}

RotaryEncoder* RotaryEncoder::instance_;
