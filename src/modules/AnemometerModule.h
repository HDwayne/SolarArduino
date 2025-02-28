#ifndef MODULE_ANENOMETER_H
#define MODULE_ANENOMETER_H

#include <Arduino.h>
#include <ezButton.h>

class AnenometerModule
{
public:
  AnenometerModule();
  void init();
  bool isTriggered();

private:
  ezButton *button;
};

#endif // MODULE_ANENOMETER_H