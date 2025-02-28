#ifndef MODULE_WIFI_H
#define MODULE_WIFI_H

#include <Arduino.h>
#include <WiFi.h>

/**
 * WifiModule
 * 
 * A class to manage Wi-Fi connection on ESP32 with manual (explicit) reconnection
 * logic, optional exponential backoff, and a maximum reconnect threshold to
 * restart the system if Wi-Fi fails consistently.
 */
class WifiModule {
public:
    WifiModule();

    /**
     * @brief Initializes Wi-Fi.
     * 
     * - Reads SSID and password from some config module (not shown here).
     * - Disables auto-reconnect for a more predictable manual approach.
     * - Sets station mode, registers the event callback, and attempts the first connection.
     */
    void init();

    /**
     * @brief Must be called regularly (e.g., from the Arduino loop() or a dedicated FreeRTOS task).
     * 
     * - Checks Wi-Fi status.
     * - If disconnected, attempts reconnection at intervals determined by an internal timer.
     * - Supports an exponential backoff (optional) to avoid spamming the network.
     * - May trigger an ESP.restart() after too many consecutive failures.
     */
    void loop();

    /**
     * @brief Returns the device’s MAC address (6 bytes).
     */
    byte* getMacAddr() { return macAddr; }

private:
    const char* ssid;
    const char* password;

    byte macAddr[6];
    unsigned long lastReconnectAttempt;
    unsigned long reconnectDelay;
    int failureCount;

    static constexpr int maxReconnectAttempts = 10;  // After x fails, do ESP.restart()

  static void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
};

#endif // MODULE_WIFI_H