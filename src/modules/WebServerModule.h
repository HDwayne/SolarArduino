#if defined(ESP32)

#ifndef MODULE_WEBSERVER_H
#define MODULE_WEBSERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "ConfigModule.h"
#include "utils/Logger.h"

class WebServerModule
{
public:
  WebServerModule();
  void begin();
  bool isRestartRequested() { return restartRequested; }

  void broadcastMessage(const String &message);

private:
  volatile bool restartRequested = false;

  void initSPIFFS();
  void setupServer();

  AsyncWebServer server;
  AsyncWebSocket ws;

  void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data, size_t len);

  void handleRestartSystem(AsyncWebServerRequest *request);

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

  void handleResetPinsConfig(AsyncWebServerRequest *request);
  void handleResetConstantsConfig(AsyncWebServerRequest *request);
  void handleResetSolarTrackingConfig(AsyncWebServerRequest *request);
  void handleResetSolTrackOptionsConfig(AsyncWebServerRequest *request);
  void handleResetAzimuthConfig(AsyncWebServerRequest *request);
  void handleResetElevationConfig(AsyncWebServerRequest *request);
  void handleResetJoystickConfig(AsyncWebServerRequest *request);
  void handleResetAnemometerConfig(AsyncWebServerRequest *request);
  void handleResetMQTTConfig(AsyncWebServerRequest *request);
  void handleResetNTPConfig(AsyncWebServerRequest *request);
  void handleResetWiFiConfig(AsyncWebServerRequest *request);
};

extern WebServerModule webServerModule;

#endif // MODULE_WEBSERVER_H
#endif // defined(ESP32)