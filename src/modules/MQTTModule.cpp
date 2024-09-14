#if defined(ESP32)

#include "MQTTModule.h"
#include "config.h"
#include "SolTrack.h"

MQTTModuleConfig mqtttModuleConfig = {
    .mqtt_server = MQTT_SERVER,
    .mqtt_port = MQTT_PORT,
    .mqtt_user = MQTT_USER,
    .mqtt_password = MQTT_PASSWORD,
    .device_name = DEVICE_NAME};

MQTTModule mqttModule(mqtttModuleConfig);

// ----------------- Constructeur -----------------

MQTTModule::MQTTModule(const MQTTModuleConfig &config)
    : client(espClient),
      mqtt_server(config.mqtt_server),
      mqtt_port(config.mqtt_port),
      mqtt_user(config.mqtt_user),
      mqtt_password(config.mqtt_password),
      device_name(config.device_name)
{
}

// ----------------- Initialisation -----------------

void MQTTModule::init()
{
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(512);
  WiFi.macAddress(macAddr);

  // Créer un identifiant unique pour Home Assistant
  createDiscoveryUniqueID();

  // Connexion au serveur MQTT
  connect();

  // Découverte Home Assistant
  publishMQTTDiscovery();
}

// ----------------- public methods -----------------

void MQTTModule::connect()
{
  while (!client.connected())
  {
    if (client.connect(device_name, mqtt_user, mqtt_password))
    {
      Serial.println("[MQTT] connected");
    }
    else
    {
      Serial.print("[MQTT] failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void MQTTModule::loop()
{
  client.loop();
}

void MQTTModule::updateField(MQTTFields field, void *value)
{
  if (field == NUM_MQTT_FIELDS)
    return;

  char topic[128];

  sprintf(topic, "homeassistant/sensor/%s/%s/state", String(devUniqueID).c_str(), MQTTFieldsNames[field]);

  switch (field)
  {
  case SOLAR_POSITION:
  {
    STPosition *val = (STPosition *)value;
    JsonDocument doc;

    doc["azimuth"] = val->azimuthRefract;
    doc["altitude"] = val->altitudeRefract;

    char buffer[512];
    serializeJson(doc, buffer);

    Serial.print("[MQTT] Update message: ");
    Serial.println(buffer);

    client.publish(topic, buffer, true);
    break;
  }
  case LAST_PANEL_ADJUSTMENT_TIME:
  case NEXT_PANEL_ADJUSTMENT_TIME:
  case PANEL_STATUS:
  case PANEL_POSITION:
  default:
    break;
  }
}

// ----------------- private methods -----------------

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
  Serial.print("Unique ID: ");
  Serial.println(devUniqueID);
}

void MQTTModule::publishMQTTDiscovery()
{
  for (int i = 0; i < NUM_MQTT_FIELDS; i++)
  {
    JsonDocument doc;

    doc["name"] = MQTTFieldsNames[i];
    doc["state_topic"] = "homeassistant/sensor/" + String(devUniqueID) + "/" + MQTTFieldsNames[i] + "/state";
    doc["unique_id"] = String(devUniqueID) + "_" + MQTTFieldsTopics[i];
    doc["device_class"] = MQTTFieldsClasses[i];
    doc["icon"] = MQTTFieldsIcons[i];
    doc["unit_of_measurement"] = MQTTFieldsUnits[i];
    doc["device"]["identifiers"] = devUniqueID;
    doc["device"]["name"] = "Solar Tracker";
    doc["device"]["model"] = "ESP32 Solar Tracker";
    doc["device"]["manufacturer"] = "Home1744";

    // Convertir le document JSON en chaîne
    char buffer[512];
    serializeJson(doc, buffer);

    Serial.print("[MQTT] Discovery message: ");
    Serial.println(buffer);

    // Définir le sujet pour MQTT Discovery (configuration Home Assistant)
    String configTopic = "homeassistant/sensor/" + String(devUniqueID) + "/" + MQTTFieldsNames[i] + "/config";

    Serial.print("[MQTT] Discovery topic: ");
    Serial.println(configTopic);

    // Publier le message sur le sujet MQTT Discovery
    if (!(client.publish(configTopic.c_str(), buffer, true)))
    {
      Serial.println("[MQTT] Failed to publish MQTT Discovery message");
    }
  }
}

#endif