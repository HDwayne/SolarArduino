// MODULE_WIFI_H

#if defined(ESP32)
#ifndef MODULE_WIFI_H
#define MODULE_WIFI_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

struct WifiModuleConfig
{
  const char *ssid;
  const char *password;
};

class WifiModule
{
public:
  WifiModule(const WifiModuleConfig &ModuleConfig);

  void init();
  byte *getMacAddr();

private:
  const char *ssid;
  const char *password;

  byte macAddr[6];
};

extern WifiModule wifiModule;

#endif // MODULE_WIFI_H
#endif // defined(ESP32)