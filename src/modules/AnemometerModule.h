// MODULE_RTC_H

#ifndef MODULE_ANENOMETER_H
#define MODULE_ANENOMETER_H

#include <Arduino.h>
#include <ezButton.h>

struct AnenometerModuleConfig
{
  uint8_t buttonPin;
};

class AnenometerModule
{
public:
  AnenometerModule(const AnenometerModuleConfig &config);

  bool isTriggered();

private:
  ezButton *button;
};

extern AnenometerModule anenometerModule;

#endif // MODULE_ANENOMETER_H