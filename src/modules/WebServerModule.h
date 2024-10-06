#if defined(ESP32)

#ifndef CONFIG_WEBSERVER_MODULE_H
#define CONFIG_WEBSERVER_MODULE_H

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
  void handleSaveConfig(AsyncWebServerRequest *request);
};

extern WebServerModule webServerModule;

#endif // CONFIG_WEBSERVER_MODULE_H
#endif // defined(ESP32)