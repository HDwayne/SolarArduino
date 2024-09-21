#include "AnemometerModule.h"
#include "config.h"

AnenometerModuleConfig anenometerConfig = {
    ANENOMETER_BUTTON_PIN};

AnenometerModule anenometerModule(anenometerConfig);

// ----------------- Anenometer Module Constructor -----------------

AnenometerModule::AnenometerModule(const AnenometerModuleConfig &config)
{
  button = new ezButton(config.buttonPin);
  button->setDebounceTime(250);
}

// ----------------- Anenometer Module Loop -----------------

bool AnenometerModule::isTriggered()
{
  button->loop();
  return button->isPressed();
}
