#if defined(ESP32)

#include "WifiModule.h"
#include "credentials.h"

WifiModuleConfig wifiModuleConfig = {
    WIFI_SSID,
    WIFI_PASSWORD,
};

WifiModule wifiModule(wifiModuleConfig);

// ----------------- Wifi Module Constructor -----------------

WifiModule::WifiModule(const WifiModuleConfig &ModuleConfig)
{
  ssid = ModuleConfig.ssid;
  password = ModuleConfig.password;
}

// ----------------- Wifi control functions -----------------

void WifiModule::init()
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  WiFi.macAddress(macAddr);
  WiFi.onEvent(WiFiEvent);
}

void WifiModule::WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("[WiFi] connected with IP: " + WiFi.localIP().toString());
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("[WiFi] Disconnected from Wi-Fi, ESP32 will try to reconnect");
    break;
  default:
    break;
  }
}

#endif // defined(ESP32)