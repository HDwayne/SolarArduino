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

byte *WifiModule::getMacAddr()
{
  return macAddr;
}

void WifiModule::init()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.macAddress(macAddr);
  WiFi.setAutoReconnect(true);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("[WiFi] Connected to AP");
  Serial.print("\tIP Address: ");
  Serial.println(WiFi.localIP());
}

#endif // defined(ESP32)