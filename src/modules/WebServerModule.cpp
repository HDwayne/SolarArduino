#if defined(ESP32)

#include "WebServerModule.h"
#include <SPIFFS.h>
#include <WiFi.h>

WebServerModule webServerModule;

WebServerModule::WebServerModule() : server(80) {}

void WebServerModule::begin()
{
  initSPIFFS();
  setupServer();
}

void WebServerModule::initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
  }
  else
  {
    Serial.println("SPIFFS mounted successfully");
  }
}

void WebServerModule::setupServer()
{
  server.on("/config", HTTP_GET, std::bind(&WebServerModule::handleConfigPage, this, std::placeholders::_1));

  server.on("/saveConfig", HTTP_POST, std::bind(&WebServerModule::handleSaveConfig, this, std::placeholders::_1));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("config.html");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->redirect("/config"); });

  server.begin();
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

  // Solar Track Options
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
  // html.replace("{{mqttPassword}}", configModule.getMQTTPassword());
  html.replace("{{deviceName}}", configModule.getDeviceName());

  // NTP Settings
  html.replace("{{ntpServer1}}", configModule.getNTPServer1());
  html.replace("{{ntpServer2}}", configModule.getNTPServer2());
  html.replace("{{ntpServer3}}", configModule.getNTPServer3());

  // Wi-Fi Credentials
  html.replace("{{wifiSSID}}", configModule.getWIFISSID());
  // html.replace("{{wifiPassword}}", configModule.getWIFIPassword());

  request->send(200, "text/html", html);
}

void WebServerModule::handleSaveConfig(AsyncWebServerRequest *request)
{
  int params = request->params();
  for (int i = 0; i < params; i++)
  {
    AsyncWebParameter *p = request->getParam(i);
    const char *paramName = p->name().c_str();
    const char *paramValue = p->value().c_str();

    // PINS
    if (strcmp(paramName, "azimuthMotorPinEn") == 0)
    {
      configModule.setAzimuthMotorPinEn(atoi(paramValue));
    }
    else if (strcmp(paramName, "azimuthMotorPwmPinL") == 0)
    {
      configModule.setAzimuthMotorPwmPinL(atoi(paramValue));
    }
    else if (strcmp(paramName, "azimuthMotorPwmPinR") == 0)
    {
      configModule.setAzimuthMotorPwmPinR(atoi(paramValue));
    }
    else if (strcmp(paramName, "azimuthLimitSwitchPin") == 0)
    {
      configModule.setAzimuthLimitSwitchPin(atoi(paramValue));
    }
    else if (strcmp(paramName, "elevationMotorPinEn") == 0)
    {
      configModule.setElevationMotorPinEn(atoi(paramValue));
    }
    else if (strcmp(paramName, "elevationMotorPwmPinU") == 0)
    {
      configModule.setElevationMotorPwmPinU(atoi(paramValue));
    }
    else if (strcmp(paramName, "elevationMotorPwmPinD") == 0)
    {
      configModule.setElevationMotorPwmPinD(atoi(paramValue));
    }
    else if (strcmp(paramName, "joystickVrxPin") == 0)
    {
      configModule.setJoystickVrxPin(atoi(paramValue));
    }
    else if (strcmp(paramName, "joystickVryPin") == 0)
    {
      configModule.setJoystickVryPin(atoi(paramValue));
    }
    else if (strcmp(paramName, "joystickButtonPin") == 0)
    {
      configModule.setJoystickButtonPin(atoi(paramValue));
    }
    else if (strcmp(paramName, "anenometerButtonPin") == 0)
    {
      configModule.setAnenometerButtonPin(atoi(paramValue));
    }

    // Constants
    else if (strcmp(paramName, "updatePanelAdjustmentInterval") == 0)
    {
      configModule.setUpdatePanelAdjustmentInterval(atoi(paramValue));
    }

    // Solar Tracking Settings
    else if (strcmp(paramName, "stLatitude") == 0)
    {
      configModule.setSTLatitude(atof(paramValue));
    }
    else if (strcmp(paramName, "stLongitude") == 0)
    {
      configModule.setSTLongitude(atof(paramValue));
    }
    else if (strcmp(paramName, "stPressure") == 0)
    {
      configModule.setSTPressure(atof(paramValue));
    }
    else if (strcmp(paramName, "stTemperature") == 0)
    {
      configModule.setSTTemperature(atof(paramValue));
    }

    // Solar Track Options (Booleans)
    else if (strcmp(paramName, "useDegrees") == 0)
    {
      configModule.setUseDegrees(true);
    }
    else if (strcmp(paramName, "useNorthEqualsZero") == 0)
    {
      configModule.setUseNorthEqualsZero(true);
    }
    else if (strcmp(paramName, "computeRefrEquatorial") == 0)
    {
      configModule.setComputeRefrEquatorial(true);
    }
    else if (strcmp(paramName, "computeDistance") == 0)
    {
      configModule.setComputeDistance(true);
    }

    // Azimuth Settings
    else if (strcmp(paramName, "azimuthMotorPWMSpeed") == 0)
    {
      configModule.setAzimuthMotorPWMSpeed(atoi(paramValue));
    }
    else if (strcmp(paramName, "azimuthDegMax") == 0)
    {
      configModule.setAzimuthDegMax(atof(paramValue));
    }
    else if (strcmp(paramName, "azimuthDegMin") == 0)
    {
      configModule.setAzimuthDegMin(atof(paramValue));
    }
    else if (strcmp(paramName, "azimuthTimeThreshold") == 0)
    {
      configModule.setAzimuthTimeThreshold(atoi(paramValue));
    }
    else if (strcmp(paramName, "azimuthTimeMaxBeforeCalibration") == 0)
    {
      configModule.setAzimuthTimeMaxBeforeCalibration(atoi(paramValue));
    }

    // Elevation Settings
    else if (strcmp(paramName, "elevationMotorPWMSpeed") == 0)
    {
      configModule.setElevationMotorPWMSpeed(atoi(paramValue));
    }
    else if (strcmp(paramName, "elevationDegMax") == 0)
    {
      configModule.setElevationDegMax(atof(paramValue));
    }
    else if (strcmp(paramName, "elevationDegMin") == 0)
    {
      configModule.setElevationDegMin(atof(paramValue));
    }
    else if (strcmp(paramName, "elevationTimeThreshold") == 0)
    {
      configModule.setElevationTimeThreshold(atoi(paramValue));
    }
    else if (strcmp(paramName, "elevationActuatorSpeed") == 0)
    {
      configModule.setElevationActuatorSpeed(atof(paramValue));
    }
    else if (strcmp(paramName, "elevationActuatorLength") == 0)
    {
      configModule.setElevationActuatorLength(atof(paramValue));
    }
    else if (strcmp(paramName, "forceTimeFullTravel") == 0)
    {
      configModule.setForceTimeFullTravel(atoi(paramValue));
    }

    // Joystick Settings
    else if (strcmp(paramName, "joystickButtonDebounce") == 0)
    {
      configModule.setJoystickButtonDebounce(atoi(paramValue));
    }
    else if (strcmp(paramName, "joystickThreshold") == 0)
    {
      configModule.setJoystickThreshold(atoi(paramValue));
    }

    // Anemometer Settings
    else if (strcmp(paramName, "anenometerSafeDuration") == 0)
    {
      configModule.setAnenometerSafeDuration(atol(paramValue));
    }

    // MQTT Settings
    else if (strcmp(paramName, "deviceName") == 0)
    {
      configModule.setDeviceName(paramValue);
    }
    else if (strcmp(paramName, "mqttServer") == 0)
    {
      configModule.setMQTTServer(paramValue);
    }
    else if (strcmp(paramName, "mqttPort") == 0)
    {
      configModule.setMQTTPort(atoi(paramValue));
    }
    else if (strcmp(paramName, "mqttUser") == 0)
    {
      configModule.setMQTTUser(paramValue);
    }
    else if (strcmp(paramName, "mqttPassword") == 0)
    {
      configModule.setMQTTPassword(paramValue);
    }

    // NTP Settings
    else if (strcmp(paramName, "ntpServer1") == 0)
    {
      configModule.setNTPServer1(paramValue);
    }
    else if (strcmp(paramName, "ntpServer2") == 0)
    {
      configModule.setNTPServer2(paramValue);
    }
    else if (strcmp(paramName, "ntpServer3") == 0)
    {
      configModule.setNTPServer3(paramValue);
    }

    // Wi-Fi Credentials
    else if (strcmp(paramName, "wifiSSID") == 0)
    {
      configModule.setWIFISSID(paramValue);
    }
    else if (strcmp(paramName, "wifiPassword") == 0)
    {
      configModule.setWIFIPassword(paramValue);
    }
  }

  configModule.setUseDegrees(request->hasParam("useDegrees"));
  configModule.setUseNorthEqualsZero(request->hasParam("useNorthEqualsZero"));
  configModule.setComputeRefrEquatorial(request->hasParam("computeRefrEquatorial"));
  configModule.setComputeDistance(request->hasParam("computeDistance"));

  configModule.saveConfig();
  request->redirect("/config");
}

#endif // defined(ESP32)