#ifndef MODULE_OTA_H
#define MODULE_OTA_H

#include <Arduino.h>

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

#endif // MODULE_OTA_H