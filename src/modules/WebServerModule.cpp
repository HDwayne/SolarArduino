#include "WebServerModule.h"

#include <SPIFFS.h>
#include <WiFi.h>

#include "controllers/AzimuthController.h"
#include "controllers/ElevationController.h"
#include "ConfigModule.h"
#include "utils/Logger.h"

extern AzimuthController azimuthController;
extern ElevationController elevationController;
extern ConfigModule configModule;
extern Logger Log;

WebServerModule::WebServerModule() : server(80), ws("/ws") {}

void WebServerModule::begin()
{
  initSPIFFS();
  setupServer();
}

void WebServerModule::initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Log.println("An Error has occurred while mounting SPIFFS");
  }
  else
  {
    Log.println("SPIFFS mounted successfully");
  }
}

void WebServerModule::setupServer()
{
  server.on("/", HTTP_GET, std::bind(&WebServerModule::handleConfigPage, this, std::placeholders::_1));

  server.on("/savePinsConfig", HTTP_POST, std::bind(&WebServerModule::handleSavePinsConfig, this, std::placeholders::_1));
  server.on("/saveConstantsConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveConstantsConfig, this, std::placeholders::_1));
  server.on("/saveSolarTrackingConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveSolarTrackingConfig, this, std::placeholders::_1));
  server.on("/saveSolTrackOptions", HTTP_POST, std::bind(&WebServerModule::handleSaveSolTrackOptions, this, std::placeholders::_1));
  server.on("/saveAzimuthConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveAzimuthConfig, this, std::placeholders::_1));
  server.on("/saveElevationConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveElevationConfig, this, std::placeholders::_1));
  server.on("/saveJoystickConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveJoystickConfig, this, std::placeholders::_1));
  server.on("/saveAnemometerConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveAnemometerConfig, this, std::placeholders::_1));
  server.on("/saveMQTTConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveMQTTConfig, this, std::placeholders::_1));
  server.on("/saveNTPConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveNTPConfig, this, std::placeholders::_1));
  server.on("/saveWiFiConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveWiFiConfig, this, std::placeholders::_1));

  server.on("/resetPinsConfig", HTTP_POST, std::bind(&WebServerModule::handleResetPinsConfig, this, std::placeholders::_1));
  server.on("/resetConstantsConfig", HTTP_POST, std::bind(&WebServerModule::handleResetConstantsConfig, this, std::placeholders::_1));
  server.on("/resetSolarTrackingConfig", HTTP_POST, std::bind(&WebServerModule::handleResetSolarTrackingConfig, this, std::placeholders::_1));
  server.on("/resetSolTrackOptionsConfig", HTTP_POST, std::bind(&WebServerModule::handleResetSolTrackOptionsConfig, this, std::placeholders::_1));
  server.on("/resetAzimuthConfig", HTTP_POST, std::bind(&WebServerModule::handleResetAzimuthConfig, this, std::placeholders::_1));
  server.on("/resetElevationConfig", HTTP_POST, std::bind(&WebServerModule::handleResetElevationConfig, this, std::placeholders::_1));
  server.on("/resetJoystickConfig", HTTP_POST, std::bind(&WebServerModule::handleResetJoystickConfig, this, std::placeholders::_1));
  server.on("/resetAnemometerConfig", HTTP_POST, std::bind(&WebServerModule::handleResetAnemometerConfig, this, std::placeholders::_1));
  server.on("/resetMQTTConfig", HTTP_POST, std::bind(&WebServerModule::handleResetMQTTConfig, this, std::placeholders::_1));
  server.on("/resetNTPConfig", HTTP_POST, std::bind(&WebServerModule::handleResetNTPConfig, this, std::placeholders::_1));
  server.on("/resetWiFiConfig", HTTP_POST, std::bind(&WebServerModule::handleResetWiFiConfig, this, std::placeholders::_1));

  // Handler for system restart
  server.on("/restartSystem", HTTP_POST, std::bind(&WebServerModule::handleRestartSystem, this, std::placeholders::_1));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("config.html");

  ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client,
                    AwsEventType type, void *arg, uint8_t *data, size_t len)
             { this->onWebSocketEvent(server, client, type, arg, data, len); });

  server.addHandler(&ws);

  server.begin();
}

void WebServerModule::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                       AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Log.printf("WebSocket client #%u connected from %s\n",
               client->id(), client->remoteIP().toString().c_str());
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Log.printf("WebSocket client #%u disconnected\n", client->id());
  }
  // else if (type == WS_EVT_DATA)
  // {
  // Traitez les données reçues
  // }
}

void WebServerModule::broadcastMessage(const String &message)
{
  ws.textAll(message);
}

void WebServerModule::handleConfigPage(AsyncWebServerRequest *request)
{
  File file = SPIFFS.open("/config.html", "r");
  if (!file)
  {
    request->send(500, "text/plain", "Failed to open config.html");
    return;
  }

  String html = file.readString();
  file.close();

  // Replace placeholders with actual values

  // Overview placeholders
  html.replace("{{currentAzimuth}}", String(azimuthController.getCurrentAzimuth(), 2));
  html.replace("{{currentElevation}}", String(elevationController.getCurrentElevation(), 2));

  // PINS
  html.replace("{{azimuthMotorPinEn}}", String(configModule.getAzimuthMotorPinEn()));
  html.replace("{{azimuthMotorPwmPinL}}", String(configModule.getAzimuthMotorPwmPinL()));
  html.replace("{{azimuthMotorPwmPinR}}", String(configModule.getAzimuthMotorPwmPinR()));
  html.replace("{{azimuthLimitSwitchPin}}", String(configModule.getAzimuthLimitSwitchPin()));

  html.replace("{{elevationMotorPinEn}}", String(configModule.getElevationMotorPinEn()));
  html.replace("{{elevationMotorPwmPinU}}", String(configModule.getElevationMotorPwmPinU()));
  html.replace("{{elevationMotorPwmPinD}}", String(configModule.getElevationMotorPwmPinD()));

  html.replace("{{joystickVrxPin}}", String(configModule.getJoystickVrxPin()));
  html.replace("{{joystickVryPin}}", String(configModule.getJoystickVryPin()));
  html.replace("{{joystickButtonPin}}", String(configModule.getJoystickButtonPin()));

  html.replace("{{anenometerButtonPin}}", String(configModule.getAnenometerButtonPin()));

  // Constants
  html.replace("{{updatePanelAdjustmentInterval}}", String(configModule.getUpdatePanelAdjustmentInterval()));

  // Solar Tracking Settings
  html.replace("{{stLatitude}}", String(configModule.getSTLatitude(), 6));
  html.replace("{{stLongitude}}", String(configModule.getSTLongitude(), 6));
  html.replace("{{stPressure}}", String(configModule.getSTPressure(), 2));
  html.replace("{{stTemperature}}", String(configModule.getSTTemperature(), 2));

  // SolTrack Options
  html.replace("{{useDegreesChecked}}", configModule.getUseDegrees() ? "checked" : "");
  html.replace("{{useNorthEqualsZeroChecked}}", configModule.getUseNorthEqualsZero() ? "checked" : "");
  html.replace("{{computeRefrEquatorialChecked}}", configModule.getComputeRefrEquatorial() ? "checked" : "");
  html.replace("{{computeDistanceChecked}}", configModule.getComputeDistance() ? "checked" : "");

  // Azimuth Settings
  html.replace("{{azimuthMotorPWMSpeed}}", String(configModule.getAzimuthMotorPWMSpeed()));
  html.replace("{{azimuthDegMax}}", String(configModule.getAzimuthDegMax(), 2));
  html.replace("{{azimuthDegMin}}", String(configModule.getAzimuthDegMin(), 2));
  html.replace("{{azimuthTimeThreshold}}", String(configModule.getAzimuthTimeThreshold()));
  html.replace("{{azimuthTimeMaxBeforeCalibration}}", String(configModule.getAzimuthTimeMaxBeforeCalibration()));

  // Elevation Settings
  html.replace("{{elevationMotorPWMSpeed}}", String(configModule.getElevationMotorPWMSpeed()));
  html.replace("{{elevationDegMax}}", String(configModule.getElevationDegMax(), 2));
  html.replace("{{elevationDegMin}}", String(configModule.getElevationDegMin(), 2));
  html.replace("{{elevationTimeThreshold}}", String(configModule.getElevationTimeThreshold()));
  html.replace("{{elevationActuatorSpeed}}", String(configModule.getElevationActuatorSpeed(), 2));
  html.replace("{{elevationActuatorLength}}", String(configModule.getElevationActuatorLength(), 2));
  html.replace("{{forceTimeFullTravel}}", String(configModule.getForceTimeFullTravel()));

  // Joystick Settings
  html.replace("{{joystickButtonDebounce}}", String(configModule.getJoystickButtonDebounce()));
  html.replace("{{joystickThreshold}}", String(configModule.getJoystickThreshold()));

  // Anemometer Settings
  html.replace("{{anenometerSafeDuration}}", String(configModule.getAnenometerSafeDuration()));

  // MQTT Settings
  html.replace("{{mqttServer}}", configModule.getMQTTServer());
  html.replace("{{mqttPort}}", String(configModule.getMQTTPort()));
  html.replace("{{mqttUser}}", configModule.getMQTTUser());
  html.replace("{{mqttPassword}}", configModule.getMQTTPassword());
  html.replace("{{deviceName}}", configModule.getDeviceName());

  // NTP Settings
  html.replace("{{ntpServer1}}", configModule.getNTPServer1());
  html.replace("{{ntpServer2}}", configModule.getNTPServer2());
  html.replace("{{ntpServer3}}", configModule.getNTPServer3());

  // Wi-Fi Credentials
  html.replace("{{wifiSSID}}", configModule.getWIFISSID());
  html.replace("{{wifiPassword}}", configModule.getWIFIPassword());

  request->send(200, "text/html", html);
}

void WebServerModule::handleRestartSystem(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", "System is restarting...");
  delay(1000);
  restartRequested = true;
}

// Individual handlers for saving configurations
void WebServerModule::handleSavePinsConfig(AsyncWebServerRequest *request)
{
  // PINS
  if (request->hasParam("azimuthMotorPinEn", true))
  {
    configModule.setAzimuthMotorPinEn(request->getParam("azimuthMotorPinEn", true)->value().toInt());
  }
  if (request->hasParam("azimuthMotorPwmPinL", true))
  {
    configModule.setAzimuthMotorPwmPinL(request->getParam("azimuthMotorPwmPinL", true)->value().toInt());
  }
  if (request->hasParam("azimuthMotorPwmPinR", true))
  {
    configModule.setAzimuthMotorPwmPinR(request->getParam("azimuthMotorPwmPinR", true)->value().toInt());
  }
  if (request->hasParam("azimuthLimitSwitchPin", true))
  {
    configModule.setAzimuthLimitSwitchPin(request->getParam("azimuthLimitSwitchPin", true)->value().toInt());
  }
  if (request->hasParam("elevationMotorPinEn", true))
  {
    configModule.setElevationMotorPinEn(request->getParam("elevationMotorPinEn", true)->value().toInt());
  }
  if (request->hasParam("elevationMotorPwmPinU", true))
  {
    configModule.setElevationMotorPwmPinU(request->getParam("elevationMotorPwmPinU", true)->value().toInt());
  }
  if (request->hasParam("elevationMotorPwmPinD", true))
  {
    configModule.setElevationMotorPwmPinD(request->getParam("elevationMotorPwmPinD", true)->value().toInt());
  }
  if (request->hasParam("joystickVrxPin", true))
  {
    configModule.setJoystickVrxPin(request->getParam("joystickVrxPin", true)->value().toInt());
  }
  if (request->hasParam("joystickVryPin", true))
  {
    configModule.setJoystickVryPin(request->getParam("joystickVryPin", true)->value().toInt());
  }
  if (request->hasParam("joystickButtonPin", true))
  {
    configModule.setJoystickButtonPin(request->getParam("joystickButtonPin", true)->value().toInt());
  }
  if (request->hasParam("anenometerButtonPin", true))
  {
    configModule.setAnenometerButtonPin(request->getParam("anenometerButtonPin", true)->value().toInt());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveConstantsConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("updatePanelAdjustmentInterval", true))
  {
    configModule.setUpdatePanelAdjustmentInterval(request->getParam("updatePanelAdjustmentInterval", true)->value().toInt());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveSolarTrackingConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("stLatitude", true))
  {
    configModule.setSTLatitude(request->getParam("stLatitude", true)->value().toFloat());
  }
  if (request->hasParam("stLongitude", true))
  {
    configModule.setSTLongitude(request->getParam("stLongitude", true)->value().toFloat());
  }
  if (request->hasParam("stPressure", true))
  {
    configModule.setSTPressure(request->getParam("stPressure", true)->value().toFloat());
  }
  if (request->hasParam("stTemperature", true))
  {
    configModule.setSTTemperature(request->getParam("stTemperature", true)->value().toFloat());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveSolTrackOptions(AsyncWebServerRequest *request)
{
  configModule.setUseDegrees(request->hasParam("useDegrees", true));
  configModule.setUseNorthEqualsZero(request->hasParam("useNorthEqualsZero", true));
  configModule.setComputeRefrEquatorial(request->hasParam("computeRefrEquatorial", true));
  configModule.setComputeDistance(request->hasParam("computeDistance", true));

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveAzimuthConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("azimuthMotorPWMSpeed", true))
  {
    configModule.setAzimuthMotorPWMSpeed(request->getParam("azimuthMotorPWMSpeed", true)->value().toInt());
  }
  if (request->hasParam("azimuthDegMax", true))
  {
    configModule.setAzimuthDegMax(request->getParam("azimuthDegMax", true)->value().toFloat());
  }
  if (request->hasParam("azimuthDegMin", true))
  {
    configModule.setAzimuthDegMin(request->getParam("azimuthDegMin", true)->value().toFloat());
  }
  if (request->hasParam("azimuthTimeThreshold", true))
  {
    configModule.setAzimuthTimeThreshold(request->getParam("azimuthTimeThreshold", true)->value().toInt());
  }
  if (request->hasParam("azimuthTimeMaxBeforeCalibration", true))
  {
    configModule.setAzimuthTimeMaxBeforeCalibration(request->getParam("azimuthTimeMaxBeforeCalibration", true)->value().toInt());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveElevationConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("elevationMotorPWMSpeed", true))
  {
    configModule.setElevationMotorPWMSpeed(request->getParam("elevationMotorPWMSpeed", true)->value().toInt());
  }
  if (request->hasParam("elevationDegMax", true))
  {
    configModule.setElevationDegMax(request->getParam("elevationDegMax", true)->value().toFloat());
  }
  if (request->hasParam("elevationDegMin", true))
  {
    configModule.setElevationDegMin(request->getParam("elevationDegMin", true)->value().toFloat());
  }
  if (request->hasParam("elevationTimeThreshold", true))
  {
    configModule.setElevationTimeThreshold(request->getParam("elevationTimeThreshold", true)->value().toInt());
  }
  if (request->hasParam("elevationActuatorSpeed", true))
  {
    configModule.setElevationActuatorSpeed(request->getParam("elevationActuatorSpeed", true)->value().toFloat());
  }
  if (request->hasParam("elevationActuatorLength", true))
  {
    configModule.setElevationActuatorLength(request->getParam("elevationActuatorLength", true)->value().toFloat());
  }
  if (request->hasParam("forceTimeFullTravel", true))
  {
    configModule.setForceTimeFullTravel(request->getParam("forceTimeFullTravel", true)->value().toInt());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveJoystickConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("joystickButtonDebounce", true))
  {
    configModule.setJoystickButtonDebounce(request->getParam("joystickButtonDebounce", true)->value().toInt());
  }
  if (request->hasParam("joystickThreshold", true))
  {
    configModule.setJoystickThreshold(request->getParam("joystickThreshold", true)->value().toInt());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveAnemometerConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("anenometerSafeDuration", true))
  {
    configModule.setAnenometerSafeDuration(request->getParam("anenometerSafeDuration", true)->value().toInt());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveMQTTConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("deviceName", true))
  {
    configModule.setDeviceName(request->getParam("deviceName", true)->value().c_str());
  }
  if (request->hasParam("mqttServer", true))
  {
    configModule.setMQTTServer(request->getParam("mqttServer", true)->value().c_str());
  }
  if (request->hasParam("mqttPort", true))
  {
    configModule.setMQTTPort(request->getParam("mqttPort", true)->value().toInt());
  }
  if (request->hasParam("mqttUser", true))
  {
    configModule.setMQTTUser(request->getParam("mqttUser", true)->value().c_str());
  }
  if (request->hasParam("mqttPassword", true))
  {
    configModule.setMQTTPassword(request->getParam("mqttPassword", true)->value().c_str());
  }

  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleSaveNTPConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("ntpServer1", true))
  {
    configModule.setNTPServer1(request->getParam("ntpServer1", true)->value().c_str());
  }
  if (request->hasParam("ntpServer2", true))
  {
    configModule.setNTPServer2(request->getParam("ntpServer2", true)->value().c_str());
  }
  if (request->hasParam("ntpServer3", true))
  {
    configModule.setNTPServer3(request->getParam("ntpServer3", true)->value().c_str());
  }

  configModule.saveConfig();

  request->redirect("/");
}

void WebServerModule::handleSaveWiFiConfig(AsyncWebServerRequest *request)
{
  if (request->hasParam("wifiSSID", true))
  {
    configModule.setWIFISSID(request->getParam("wifiSSID", true)->value().c_str());
  }
  if (request->hasParam("wifiPassword", true))
  {
    configModule.setWIFIPassword(request->getParam("wifiPassword", true)->value().c_str());
  }

  configModule.saveConfig();
  request->redirect("/");
}

// Individual handlers for resetting configurations
void WebServerModule::handleResetPinsConfig(AsyncWebServerRequest *request)
{
  configModule.resetPinsConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetConstantsConfig(AsyncWebServerRequest *request)
{
  configModule.resetConstantsConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetSolarTrackingConfig(AsyncWebServerRequest *request)
{
  configModule.resetSolarTrackingConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetSolTrackOptionsConfig(AsyncWebServerRequest *request)
{
  configModule.resetSolTrackOptionsConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetAzimuthConfig(AsyncWebServerRequest *request)
{
  configModule.resetAzimuthConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetElevationConfig(AsyncWebServerRequest *request)
{
  configModule.resetElevationConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetJoystickConfig(AsyncWebServerRequest *request)
{
  configModule.resetJoystickConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetAnemometerConfig(AsyncWebServerRequest *request)
{
  configModule.resetAnemometerConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetMQTTConfig(AsyncWebServerRequest *request)
{
  configModule.resetMQTTConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetNTPConfig(AsyncWebServerRequest *request)
{
  configModule.resetNTPConfig();
  configModule.saveConfig();
  request->redirect("/");
}

void WebServerModule::handleResetWiFiConfig(AsyncWebServerRequest *request)
{
  configModule.resetWiFiConfig();
  configModule.saveConfig();
  request->redirect("/");
}