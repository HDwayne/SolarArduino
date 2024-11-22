#if defined(ESP32)

#include "WifiModule.h"
#include "ConfigModule.h"

WifiModule wifiModule;

// ----------------- Wifi Module Constructor -----------------

WifiModule::WifiModule() {}

// ----------------- Wifi control functions -----------------

void WifiModule::init()
{
  ssid = configModule.getWIFISSID();
  password = configModule.getWIFIPassword();

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
    Log.println("[WiFi] connected with IP: " + WiFi.localIP().toString());
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Log.println("[WiFi] Disconnected from Wi-Fi, ESP32 will try to reconnect");
    break;
  default:
    break;
  }
}

#endif // defined(ESP32)