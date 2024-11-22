#if defined(ESP32)

#include "MQTTModule.h"
#include "ConfigModule.h"
#include "SolTrack.h"
#include <RTClib.h>

MQTTModule mqttModule;

// ----------------- Constructeur -----------------

MQTTModule::MQTTModule() : client(espClient) {}

// ----------------- Initialisation -----------------

void MQTTModule::init()
{
  mqtt_server = configModule.getMQTTServer();
  mqtt_port = configModule.getMQTTPort();
  mqtt_user = configModule.getMQTTUser();
  mqtt_password = configModule.getMQTTPassword();
  device_name = configModule.getDeviceName();

  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(512);
  WiFi.macAddress(macAddr);
  createDiscoveryUniqueID();

  for (int i = 0; i < NUM_MQTT_FIELDS; i++)
    messages[i] = "";
}

// ----------------- public methods -----------------

void MQTTModule::loop()
{
  if (!client.connected())
  {
    long now = millis();
    if (now - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = now;
      if (reconnect())
      {
        lastReconnectAttempt = 0;
      }
    }
  }
  else
  {
    client.loop();

    for (int i = 0; i < NUM_MQTT_FIELDS; i++)
    {
      if (messages[i] != "") // Check if there's a pending message for this field
      {
        char topic[128];
        sprintf(topic, "homeassistant/sensor/%s/%s/state", String(devUniqueID).c_str(), MQTTFieldsNames[i]);

        if (client.publish(topic, messages[i].c_str(), true))
        {
          Log.print("[MQTT] Published message: ");
          Log.println(messages[i]);

          messages[i] = "";
        }
        else
        {
          Log.println("[MQTT] Failed to publish message, will retry.");
        }
      }
    }
  }
}

void MQTTModule::updateField(MQTTFields field, void *value)
{
  if (field == NUM_MQTT_FIELDS)
    return;

  switch (field)
  {
  case PANEL_AZIMUTH:
  case PANEL_ELEVATION:
  {
    float *val = (float *)value;
    char buffer[64];
    sprintf(buffer, "%.2f", *val);
    messages[field] = String(buffer);
    break;
  }
  case SOLAR_AZIMUTH:
  case SOLAR_ELEVATION:
  {
    double *val = (double *)value;
    char buffer[64];
    sprintf(buffer, "%.2f", *val);
    messages[field] = String(buffer);
    break;
  }
  case LAST_PANEL_ADJUSTMENT_TIME:
  case NEXT_PANEL_ADJUSTMENT_TIME:
  {
    DateTime *val = (DateTime *)value;
    char buffer[64];
    sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02dZ", val->year(), val->month(), val->day(), val->hour(), val->minute(), val->second());
    messages[field] = String(buffer);
    break;
  }
  case PANEL_STATUS:
  default:
    break;
  }
}

// ----------------- private methods -----------------

bool MQTTModule::reconnect()
{
  if (client.connect(device_name, mqtt_user, mqtt_password))
  {
    publishMQTTDiscovery();
  }
  return client.connected();
}

void MQTTModule::createDiscoveryUniqueID()
{
  strcpy(devUniqueID, device_name);
  int preSizeBytes = strlen(device_name);
  int j = 0;
  for (int i = 2; i >= 0; i--)
  {
    sprintf(&devUniqueID[preSizeBytes + j], "%02X", macAddr[i]);
    j += 2;
  }
  Log.print("Unique ID: ");
  Log.println(devUniqueID);
}

void MQTTModule::publishMQTTDiscovery()
{
  for (int i = 0; i < NUM_MQTT_FIELDS; i++)
  {
    JsonDocument doc;

    doc["name"] = MQTTFieldsNames[i];
    doc["state_topic"] = "homeassistant/sensor/" + String(devUniqueID) + "/" + MQTTFieldsNames[i] + "/state";
    doc["unique_id"] = String(devUniqueID) + "_" + MQTTFieldsTopics[i];
    doc["value_template"] = valueTemplates[i];
    doc["device"]["identifiers"] = devUniqueID;
    doc["device"]["name"] = "Solar Tracker";
    doc["device"]["model"] = "ESP32 Solar Tracker";
    doc["device"]["manufacturer"] = "Home1744";

    // Convertir le document JSON en chaîne
    char buffer[512];
    serializeJson(doc, buffer);

    // Définir le sujet pour MQTT Discovery (configuration Home Assistant)
    String configTopic = "homeassistant/sensor/" + String(devUniqueID) + "/" + MQTTFieldsNames[i] + "/config";

    Log.print("[MQTT] Discovery topic: ");
    Log.println(configTopic);

    // Publier le message sur le sujet MQTT Discovery
    if (!(client.publish(configTopic.c_str(), buffer, true)))
    {
      Log.println("[MQTT] Failed to publish MQTT Discovery message");
    }
  }
}

#endif