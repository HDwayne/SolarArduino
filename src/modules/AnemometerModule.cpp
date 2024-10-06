#include "AnemometerModule.h"
#include "ConfigModule.h"

AnenometerModuleConfig anenometerConfig = {
    configModule.getAnenometerButtonPin()};

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
