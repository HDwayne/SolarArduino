// MODULE_OTA_H

#if defined(ESP32)
#ifndef MODULE_OTA_H
#define MODULE_OTA_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "utils/Logger.h"

class OTAModule
{
public:
  OTAModule();

  void init();
  void loop();

private:
  const char *device_name;
  byte macAddr[6];
  char devUniqueID[64];

  void createDiscoveryUniqueID();
};

extern OTAModule otaModule;

#endif // MODULE_OTA_H
#endif // defined(ESP32)