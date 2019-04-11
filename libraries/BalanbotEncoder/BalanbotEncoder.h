#ifndef BALANBOTENCODER_H
#define BALANBOTENCODER_H

#include <Arduino.h>
#define PPR 390
const int SECTION_NUMBER = 390;

class BalanbotEncoder{
  private:
    int mInterruptPin;
    int mDirectionPin;
    int mPosition;
    int count;
  public:
    BalanbotEncoder(); 
    void SetInterruptPin(const int pin);
    void SetDirectionPin(const int pin);
    void SetPins();
    void Update();
    int GetInterruptPin();
    int GetPosition();
    int GetPPR();
    void ClearPosition();
    int GetCount();
};

#endif 