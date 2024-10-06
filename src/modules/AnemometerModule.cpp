#include "AnemometerModule.h"
#include "ConfigModule.h"

AnenometerModule anenometerModule();

// ----------------- Anenometer Module Constructor -----------------

AnenometerModule::AnenometerModule() {}

// ----------------- Anenometer Module Initialisation -----------------

void AnenometerModule::init()
{
  button = new ezButton(configModule.getAnenometerButtonPin());
  button->setDebounceTime(250);
}

// ----------------- Anenometer Module Loop -----------------

bool AnenometerModule::isTriggered()
{
  button->loop();
  return button->isPressed();
}
