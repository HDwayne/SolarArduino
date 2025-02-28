#ifndef MODULE_MQTT_H
#define MODULE_MQTT_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

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
  MQTTModule();
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
      "{{ value | float }}",       // Pour PANEL_AZIMUTH : Convertir la valeur en float
      "{{ value | float }}",       // Pour PANEL_ELEVATION : Convertir la valeur en float
      "{{ value | float }}",       // Pour SOLAR_AZIMUTH : Convertir la valeur en float
      "{{ value | float }}",       // Pour SOLAR_ELEVATION : Convertir la valeur en float
      "{{ value | as_datetime }}", // Pour LAST_PANEL_ADJUSTMENT_TIME : Convertir la valeur en timestamp ISO 8601
      "{{ value | as_datetime }}", // Pour NEXT_PANEL_ADJUSTMENT_TIME : Convertir la valeur en timestamp ISO 8601
      ""};

  char devUniqueID[64];
  long lastReconnectAttempt = 0;

  bool reconnect();
  void createDiscoveryUniqueID();
  void publishMQTTDiscovery();
};

#endif // MODULE_MQTT_H