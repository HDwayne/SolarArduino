// MODULE_MQTT_H

#if defined(ESP32)
#ifndef MODULE_MQTT_H
#define MODULE_MQTT_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>

struct MQTTModuleConfig
{
  const char *mqtt_server;   // Adresse du serveur MQTT
  uint16_t mqtt_port;        // Port du serveur MQTT (par défaut 1883)
  const char *mqtt_user;     // Nom d'utilisateur (si requis)
  const char *mqtt_password; // Mot de passe (si requis)
  const char *device_name;   // Nom unique du périphérique
};

enum MQTTFields
{
  PANEL_POSITION,
  SOLAR_POSITION,
  LAST_PANEL_ADJUSTMENT_TIME,
  NEXT_PANEL_ADJUSTMENT_TIME,
  PANEL_STATUS,
  NUM_MQTT_FIELDS
};

class MQTTModule
{
public:
  MQTTModule(const MQTTModuleConfig &config);
  void init();
  void updateField(MQTTFields field, void *value);
  void loop();

private:
  PubSubClient client;
  byte macAddr[6];

  const char *mqtt_server;
  uint16_t mqtt_port;
  const char *mqtt_user;
  const char *mqtt_password;
  const char *device_name;

  WiFiClient espClient;

  String messages[NUM_MQTT_FIELDS];

  const char *MQTTFieldsNames[NUM_MQTT_FIELDS] = {
      "panel_position",
      "solar_position",
      "last_adjustment_time",
      "next_adjustment_time",
      "panel_status"};

  const char *MQTTFieldsUnits[NUM_MQTT_FIELDS] = {
      "",
      "",
      "",
      "",
      ""};

  const char *MQTTFieldsIcons[NUM_MQTT_FIELDS] = {
      "mdi:solar-power",
      "mdi:weather-sunset-up",
      "mdi:clock",
      "mdi:clock",
      "mdi:power"};

  const char *MQTTFieldsClasses[NUM_MQTT_FIELDS] = {
      "enum",
      "enum",
      "timestamp",
      "timestamp",
      "power"};

  const char *MQTTFieldsTopics[NUM_MQTT_FIELDS] = {
      "panel_position",
      "solar_position",
      "last_adjustment_time",
      "next_adjustment_time",
      "panel_status"};

  char devUniqueID[64];
  long lastReconnectAttempt = 0;

  bool reconnect();
  void createDiscoveryUniqueID();
  void publishMQTTDiscovery();
};

extern MQTTModule mqttModule;

#endif // MODULE_MQTT_H
#endif // defined(ESP32)