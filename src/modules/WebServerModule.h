#if defined(ESP32)

#ifndef MODULE_WEBSERVER_H
#define MODULE_WEBSERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "ConfigModule.h"

class WebServerModule
{
public:
  WebServerModule();
  void begin();

private:
  void initSPIFFS();
  void setupServer();

  AsyncWebServer server;

  void handleConfigPage(AsyncWebServerRequest *request);
  void handleSavePinsConfig(AsyncWebServerRequest *request);
  void handleSaveConstantsConfig(AsyncWebServerRequest *request);
  void handleSaveSolarTrackingConfig(AsyncWebServerRequest *request);
  void handleSaveSolTrackOptions(AsyncWebServerRequest *request);
  void handleSaveAzimuthConfig(AsyncWebServerRequest *request);
  void handleSaveElevationConfig(AsyncWebServerRequest *request);
  void handleSaveJoystickConfig(AsyncWebServerRequest *request);
  void handleSaveAnemometerConfig(AsyncWebServerRequest *request);
  void handleSaveMQTTConfig(AsyncWebServerRequest *request);
  void handleSaveNTPConfig(AsyncWebServerRequest *request);
  void handleSaveWiFiConfig(AsyncWebServerRequest *request);
  void handleRestartSystem(AsyncWebServerRequest *request);
};

extern WebServerModule webServerModule;

#endif // MODULE_WEBSERVER_H
#endif // defined(ESP32)