// MODULE_WIFI_H

#if defined(ESP32)
#ifndef MODULE_WIFI_H
#define MODULE_WIFI_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "utils/Logger.h"

class WifiModule
{
public:
  WifiModule();

  void init();
  byte *getMacAddr() { return macAddr; }

private:
  const char *ssid;
  const char *password;
  byte macAddr[6];

  static void WiFiEvent(WiFiEvent_t event);
};

extern WifiModule wifiModule;

#endif // MODULE_WIFI_H
#endif // defined(ESP32)