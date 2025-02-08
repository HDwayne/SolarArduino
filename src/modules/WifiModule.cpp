#if defined(ESP32)
#include "WifiModule.h"
#include "ConfigModule.h"

WifiModule wifiModule;

WifiModule::WifiModule()
  : ssid(nullptr),
    password(nullptr),
    lastReconnectAttempt(0),
    failureCount(0),
    reconnectDelay(5000)
{
}

void WifiModule::init() {
  ssid = configModule.getWIFISSID();
  password = configModule.getWIFIPassword();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.onEvent(handleWiFiEvent);
  WiFi.begin(ssid, password);
  WiFi.macAddress(macAddr);

  lastReconnectAttempt = millis();
  failureCount = 0;

  Log.println("[WiFi] Initialization complete. Attempting connection...");
}

void WifiModule::loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (failureCount != 0) {
      failureCount = 0;
      reconnectDelay = 5000;
      Log.println("[WiFi] Connection stable. Failure count reset.");
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastReconnectAttempt >= reconnectDelay) {
    lastReconnectAttempt = now;
    failureCount++;

    char buff[100];
    snprintf(buff, sizeof(buff),
             "[WiFi] Not connected. Attempting reconnection #%d...",
             failureCount);
    Log.println(buff);

    WiFi.disconnect();
    WiFi.begin(ssid, password);

    reconnectDelay = (reconnectDelay < 60000)
                     ? reconnectDelay * 2
                     : 60000; // cap at 1 minute

    if (failureCount >= maxReconnectAttempts) {
      Log.println("[WiFi] Maximum reconnect attempts reached. Restarting system...");
      ESP.restart();
    }
  }
}

void WifiModule::handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP: {
      IPAddress ip = IPAddress(info.got_ip.ip_info.ip.addr);
      char buff[64];
      snprintf(buff, sizeof(buff),
               "[WiFi] Connected with IP: %s",
               ip.toString().c_str());
      Log.println(buff);
      break;
    }
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Log.println("[WiFi] Disconnected from AP.");
      break;
    default:
      break;
  }
}

#endif // defined(ESP32)