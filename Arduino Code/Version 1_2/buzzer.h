/*
  A simple class to handle a buzzer
  2016-09-17
*/

#ifndef Buzzer_h
#define Buzzer_h

#include "Arduino.h"

class Buzzer
{
  public: 
    Buzzer(int pin); 
    
    void buzz(); 
    void buzz(int duration);
    void buzzBlocking(int duration_ms);
    void disable(); 
    void enable(); 
    bool isEnabled(); 
    void doUpdate(); 
      
  private: 
    int _pin; 
    int _counter; 
    bool _disabled;
};

#endif
