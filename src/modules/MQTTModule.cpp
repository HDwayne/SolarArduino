#include "MQTTModule.h"

#include <ArduinoJson.h>

#include "utils/Logger.h"
#include "controllers/AzimuthController.h"
#include "controllers/ElevationController.h"
#include "modules/RTCModule.h"

extern Logger Log;
extern ElevationController elevationController;
extern AzimuthController   azimuthController;
extern RTCModule           rtcModule;

// ----------------------------------------------------------------------

const char*   MQTTModule::_broker       = nullptr;
uint16_t      MQTTModule::_port         = 1883;
const char*   MQTTModule::_user         = nullptr;
const char*   MQTTModule::_password     = nullptr;

WiFiClient    MQTTModule::_wifiClient;
PubSubClient  MQTTModule::_mqttClient(_wifiClient);
QueueHandle_t MQTTModule::_publishQueue = nullptr;
bool          MQTTModule::_taskRunning  = false;
MqttCallback  MQTTModule::_userCallback = nullptr;

String        MQTTModule::_clientId     = "";
String        MQTTModule::_baseTopic    = "";

/**
 * Constructeur
 */
MQTTModule::MQTTModule()
{
}

/**
 * Initialisation globale du module MQTT
 */
void MQTTModule::begin(const char* broker, uint16_t port, const char* user, const char* password)
{
    Log.println("[MQTTModule] Initialisation...");

    // Stocker les infos du broker
    _broker    = broker;
    _port      = port;
    _user      = user;
    _password  = password;

    // Génération du clientId (ex: "ESP32Client-AB12CD34EF56")
    _clientId = "ESP32Client-";
    _clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

    // Base topic, ex: "solar/ESP32Client-AB12CD34EF56/"
    _baseTopic = String("solar/") + _clientId + "/";

    _mqttClient.setServer(_broker, _port);
    _mqttClient.setCallback(internalMessageCallback);

    if (_publishQueue == nullptr) {
        _publishQueue = xQueueCreate(10, sizeof(MqttMessage));
    }

    if (!_taskRunning && _publishQueue != nullptr)
    {
        xTaskCreatePinnedToCore(
            mqttTask,
            "MQTTTask",
            8192,
            this,
            1,
            NULL,
            0
        );
        _taskRunning = true;
    }

    Log.println("[MQTTModule] Initialisation terminée.");
}

/**
 * Publication d'un message MQTT.
 */
void MQTTModule::publish(const String& topic, const String& payload, bool retained)
{
    queuePublish(topic, payload, retained);
}

/**
 * Méthode interne : poste le message dans la queue pour un envoi asynchrone.
 */
void MQTTModule::queuePublish(const String& topic, const String& payload, bool retained)
{
    MqttMessage msg;

    strlcpy(msg.topic, topic.c_str(), MqttMessage::TOPIC_SIZE);
    strlcpy(msg.payload, payload.c_str(), MqttMessage::PAYLOAD_SIZE);

    msg.retained = retained;

    BaseType_t ok = xQueueSendToBack(_publishQueue, &msg, 0);
    if (ok != pdTRUE) {
        Log.println("[MQTTModule] Queue pleine, impossible de publier le message.");
    }
}


/**
 * Méthode pour définir le callback de réception
 */
void MQTTModule::setCallback(MqttCallback callback)
{
    _userCallback = callback;
}

/**
 * Boucle MQTT : gère la réception, appelée dans la tâche mqttTask()
 */
void MQTTModule::loop()
{
    _mqttClient.loop();
}

/**
 * (Re)connecte le client MQTT si nécessaire
 */
void MQTTModule::reconnect()
{
    if (WiFi.status() != WL_CONNECTED || _mqttClient.connected()) {
        return;
    }

    Log.print("[MQTTModule] Attempting MQTT connection to ");
    Log.print(_broker);
    Log.print(":");
    Log.println(_port);

    bool connected = false;

    if (_user && _password && strlen(_user) > 0 && strlen(_password) > 0) {
        connected = _mqttClient.connect(_clientId.c_str(), _user, _password);
    } else {
        connected = _mqttClient.connect(_clientId.c_str());
    }

    if (connected) {
        Log.println("[MQTTModule] Connected to MQTT broker!");
        
        // _mqttClient.subscribe((_baseTopic + "system/ping").c_str());

        publishWifi();
        publishRTCInfo();
        publishSystemInfo();
        publishAzimuthState();
        publishElevationState();

    } else {
        Log.print("[MQTTModule] Failed, rc=");
        Log.println(_mqttClient.state());
    }
}

/**
 * Callback interne pour PubSubClient, redirige vers le callback utilisateur
 */
void MQTTModule::internalMessageCallback(char* topic, byte* payload, unsigned int length)
{
    if (_userCallback) {
        _userCallback(topic, payload, length);
    }
}

/**
 * Tâche FreeRTOS dédiée au module MQTT.
 */
void MQTTModule::mqttTask(void* pvParameters)
{
    MQTTModule* instance = static_cast<MQTTModule*>(pvParameters);

    for (;;)
    {
        instance->reconnect();
        instance->loop();

        if (_mqttClient.connected())
        {
            MqttMessage msg;
            if (xQueueReceive(_publishQueue, &msg, 0) == pdTRUE) {
                bool pubOk = _mqttClient.publish(msg.topic, msg.payload, msg.retained);
                if (!pubOk) {
                    Log.println("[MQTTModule] Échec de la publication MQTT !");
                }
            }
            
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL); // ne devrait jamais arriver
}

// -----------------------------------------------------------------------------
// Méthodes "métier" de publication
// -----------------------------------------------------------------------------

void MQTTModule::publishWifi()
{
    StaticJsonDocument<256> doc;

    doc["ssid"] = WiFi.SSID();
    doc["rssi"] = WiFi.RSSI();
    doc["ip"]   = WiFi.localIP().toString();
    doc["mac"]  = WiFi.macAddress();

    char buffer[256];
    size_t len = serializeJson(doc, buffer);
    buffer[len] = '\0';

    String topic = _baseTopic + "system/wifi";
    queuePublish(topic, buffer, true);
}


/**
 * publishAzimuthController()
 *  e.g. "solar/<myDevice>/azimuth/state"
 */
void MQTTModule::publishAzimuthController(const String& status) {
    String topic = _baseTopic + "azimuth/state";
    queuePublish(topic, status, true);
}

/**
 * publishElevationController()
 *  e.g. "solar/<myDevice>/elevation/state"
 */
void MQTTModule::publishElevationController(const String& status) {
    String topic = _baseTopic + "elevation/state";
    queuePublish(topic, status, true);
}


/**
 * publishPanelStatus()
 *   e.g. "solar/<myDevice>/state"
 */
void MQTTModule::publishPanelStatus(const String& status) {
    String topic = _baseTopic + "state";
    queuePublish(topic, status, true);
}

/**
 * publishAdjustmentTimes()
 *   e.g. "solar/<myDevice>/adjustmentTimes"
 */
void MQTTModule::publishAdjustmentTimes(const DateTime& lastAdjustment, const DateTime& nextAdjustment) {
    StaticJsonDocument<256> doc;

    doc["lastAdjustment"] = dateTimeToISO(lastAdjustment, true);
    doc["nextAdjustment"] = dateTimeToISO(nextAdjustment, true);

    char buffer[256];
    size_t len = serializeJson(doc, buffer);
    buffer[len] = '\0';

    String topic = _baseTopic + "adjustmentTimes";
    queuePublish(topic, buffer, false);
}

/**
 * publishAzimuthState()
 *   e.g. "solar/<myDevice>/azimuth/position"
 */
void MQTTModule::publishAzimuthState() {
    float currentAz = azimuthController.getCurrentAzimuth();
    StaticJsonDocument<64> doc;
    doc["azimuth"] = currentAz;
    Log.println("[MQTTModule] Doc created");

    char buffer[64];
    size_t len = serializeJson(doc, buffer);
    buffer[len] = '\0';

    String topic = _baseTopic + "azimuth/position";
    queuePublish(topic, buffer, true);
}

/**
 * publishElevationState()
 *   e.g. "solar/<myDevice>/elevation/position"
 */
void MQTTModule::publishElevationState() {
    float currentEl = elevationController.getCurrentElevation();

    StaticJsonDocument<64> doc;
    doc["elevation"] = currentEl;

    char buffer[64];
    size_t len = serializeJson(doc, buffer);
    buffer[len] = '\0';

    String topic = _baseTopic + "elevation/position";
    queuePublish(topic, buffer, true);
}

/**
 * publishRTCInfo()
 *   e.g. "solar/<myDevice>/system/rtc"
 */
void MQTTModule::publishRTCInfo() {
    DateTime nowUTC   = rtcModule.getDateTimeUTC();
    DateTime nowLocal = rtcModule.getDateTimeLocal();

    StaticJsonDocument<256> doc;
    doc["utc"]   = dateTimeToISO(nowUTC, true);
    doc["local"] = dateTimeToISO(nowLocal, false);

    char buffer[256];
    size_t len = serializeJson(doc, buffer);
    buffer[len] = '\0';

    String topic = _baseTopic + "system/rtc";
    queuePublish(topic, buffer, true);
}

/**
 * publishSystemInfo()
 *   e.g. "solar/<myDevice>/system/info"
 */
void MQTTModule::publishSystemInfo() {
    StaticJsonDocument<256> doc;

    doc["deviceName"] = _clientId;
    doc["firmware"]   = "v1.0.0";
    doc["chip_id"]    = ESP.getEfuseMac();
    doc["chip_model"] = ESP.getChipModel();
    doc["chip_rev"]   = ESP.getChipRevision();

    char buffer[256];
    size_t len = serializeJson(doc, buffer);
    buffer[len] = '\0';

    String topic = _baseTopic + "system/info";
    queuePublish(topic, buffer, true);
}

/**
 * dateTimeToISO()
 *   Helper pour formater DateTime en ISO8601
 */
String MQTTModule::dateTimeToISO(const DateTime& dt, bool utc) {
    char buf[32];
    // Format : YYYY-MM-DDThh:mm:ssZ pour UTC, ou sans 'T'/'Z' si local
    if (utc) {
        snprintf(buf, sizeof(buf),
                 "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 dt.year(), dt.month(), dt.day(),
                 dt.hour(), dt.minute(), dt.second());
    } else {
        snprintf(buf, sizeof(buf),
                 "%04d-%02d-%02d %02d:%02d:%02d",
                 dt.year(), dt.month(), dt.day(),
                 dt.hour(), dt.minute(), dt.second());
    }
    return String(buf);
}
