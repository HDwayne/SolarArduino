#ifndef MODULE_MQTT_H
#define MODULE_MQTT_H

#include <Arduino.h>
#include <WiFi.h>
#include <functional>
#include <PubSubClient.h>
#include "RTCModule.h"

struct MqttMessage
{
    static const size_t TOPIC_SIZE   = 64;
    static const size_t PAYLOAD_SIZE = 256;

    char topic[TOPIC_SIZE];
    char payload[PAYLOAD_SIZE];
    bool retained;
};


/**
 * Typedef pour le callback de réception de message.
 * On reçoit en paramètres :
 * - Le topic,
 * - Le payload (tableau d’octets),
 * - La taille (length) du payload.
 */
typedef std::function<void(const char* topic, const byte* payload, unsigned int length)> MqttCallback;

class MQTTModule
{
public:
    MQTTModule();
    
    /**
     * Initialise la queue et crée la tâche FreeRTOS dédiée.
     * 
     * @param broker   Adresse ou IP du broker MQTT.
     * @param port     Port MQTT (1883 par défaut).
     * @param user     Nom d’utilisateur MQTT (optionnel).
     * @param password Mot de passe MQTT (optionnel).
     */
    void begin(const char* broker, 
               uint16_t port = 1883, 
               const char* user = nullptr, 
               const char* password = nullptr);

    /**
     * Méthode publique pour poster un message MQTT dans la file (non-bloquant).
     * Retained par défaut = false (modifiable si besoin).
     */
    void publish(const String& topic, const String& payload, bool retained = false);

    /**
     * Méthode pour définir un callback utilisateur afin de traiter les messages reçus.
     */
    void setCallback(MqttCallback callback);

    /**
     * Boucle MQTT : gère la connexion et la réception (principalement appelée par la tâche).
     */
    void loop();

    /**
     * Méthode pour vérifier l’état de connexion et se reconnecter si besoin.
     */
    void reconnect();

    /**
     * Méthodes de publication "métier" (exemples) :
     */
    void publishPanelStatus(const String& status);
    void publishElevationController(const String& status);
    void publishAzimuthController(const String& status);

    // Suppose l’usage de DateTime d'une librairie type RTClib
    void publishAdjustmentTimes(const DateTime& lastAdjustment, const DateTime& nextAdjustment);
    void publishAzimuthState();
    void publishElevationState();
    void publishRTCInfo();
    void publishSystemInfo();
    void publishWifi();

private:
    /**
     * Méthode interne qui poste le message dans la queue.
     * Retained paramétrable.
     */
    void queuePublish(const String& topic, const String& payload, bool retained);

    /**
     * Méthode statique qui sera la task FreeRTOS.
     */
    static void mqttTask(void* pvParameters);

    /**
     * Callback interne pour PubSubClient -> redirige vers le callback utilisateur.
     */
    static void internalMessageCallback(char* topic, byte* payload, unsigned int length);

private:
    static const char* _broker;
    static uint16_t    _port;
    static const char* _user;
    static const char* _password;
    static WiFiClient      _wifiClient;
    static PubSubClient    _mqttClient;
    static QueueHandle_t   _publishQueue;
    static bool            _taskRunning;
    static MqttCallback    _userCallback;
    static String          _clientId;
    static String          _baseTopic;

    String dateTimeToISO(const DateTime& dt, bool utc);
};

#endif // MODULE_MQTT_H
