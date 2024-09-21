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
  PANEL_AZIMUTH,
  PANEL_ELEVATION,
  SOLAR_AZIMUTH,
  SOLAR_ELEVATION,
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
      "panel_azimuth",
      "panel_elevation",
      "solar_azimuth",
      "solar_elevation",
      "last_adjustment_time",
      "next_adjustment_time",
      "panel_status"};

  const char *MQTTFieldsTopics[NUM_MQTT_FIELDS] = {
      "panel_azimuth",
      "panel_elevation",
      "solar_azimuth",
      "solar_elevation",
      "last_adjustment_time",
      "next_adjustment_time",
      "panel_status"};

  const char *valueTemplates[NUM_MQTT_FIELDS] = {
      "{{ value | float }}",        // Pour PANEL_AZIMUTH : Convertir la valeur en float
      "{{ value | float }}",        // Pour PANEL_ELEVATION : Convertir la valeur en float
      "{{ value | float }}",        // Pour SOLAR_AZIMUTH : Convertir la valeur en float
      "{{ value | float }}",        // Pour SOLAR_ELEVATION : Convertir la valeur en float
      "{{ value | as_timestamp }}", // Pour LAST_PANEL_ADJUSTMENT_TIME : Convertir la valeur en timestamp ISO 8601
      "{{ value | as_timestamp }}", // Pour NEXT_PANEL_ADJUSTMENT_TIME : Convertir la valeur en timestamp ISO 8601
      ""};

  char devUniqueID[64];
  long lastReconnectAttempt = 0;

  bool reconnect();
  void createDiscoveryUniqueID();
  void publishMQTTDiscovery();
};

extern MQTTModule mqttModule;

#endif // MODULE_MQTT_H
#endif // defined(ESP32)