/*
  A simple class to handle a buzzer
  2016-09-17
*/

#include "Arduino.h"
#include "buzzer.h"

Buzzer::Buzzer(int pin) {
  _pin = pin; 
  _counter = 0; 
  _disabled = false; 
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW); 
}

void Buzzer::buzz() {
  if (!_disabled)
    _counter = 1; 
}

void Buzzer::buzz(int duration) {
  if (!_disabled)
    _counter = duration; 
}

void Buzzer::buzzBlocking(int duration_ms) {
  if (!_disabled) {
    digitalWrite(_pin, HIGH); 
    delay(duration_ms); 
    digitalWrite(_pin, LOW);
  }
}

void Buzzer::disable() {
  _disabled = true;
}

void Buzzer::enable() {
  _disabled = false;
}

bool Buzzer::isEnabled() {
  return !_disabled; 
}

void Buzzer::doUpdate() {
  if (_counter <= 0) {
    digitalWrite(_pin, LOW);
  } else {
    _counter--; 
    digitalWrite(_pin, HIGH);    
  } 
}









