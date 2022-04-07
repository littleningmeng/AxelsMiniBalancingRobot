/*
  A basic PID class. 
  by Axel Brinkeby.
  2016-05-08
*/

#ifndef Pid_h
#define Pid_h

#include "Arduino.h"

class Pid
{
  public: 
    Pid(float p, float i, float d, float iLimit); 
    float updatePID(float target, float current, float deltaTime); 
    void resetPID(); 
    
    void setP(float p); 
    void setI(float i); 
    void setD(float d); 
    void setIlimit(float limit); 

    float getP(); 
    float getI(); 
    float getD(); 
    float getIlimit();     

  private: 
    float _P; 
    float _I; 
    float _D; 
    float _I_limit; 
    float _integratedError; 
    float _lastError; 
};

#endif
