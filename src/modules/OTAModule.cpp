#if defined(ESP32)

#include "OTAModule.h"
#include "config.h"

OTAModuleConfig otaModuleConfig = {
    .device_name = DEVICE_NAME};

OTAModule otaModule(otaModuleConfig);

// ----------------- OTA Module Constructor -----------------

OTAModule::OTAModule(const OTAModuleConfig &config)
    : device_name(config.device_name)
{
  WiFi.macAddress(macAddr);
  createDiscoveryUniqueID();
}

// ----------------- OTA control functions -----------------

void OTAModule::init()
{
  ArduinoOTA.setHostname(devUniqueID);

  ArduinoOTA
      .onStart([]()
               {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed"); });

  ArduinoOTA.begin();
}

void OTAModule::loop()
{
  ArduinoOTA.handle();
}

void OTAModule::createDiscoveryUniqueID()
{
  strcpy(devUniqueID, device_name);
  int preSizeBytes = strlen(device_name);
  int j = 0;
  for (int i = 2; i >= 0; i--)
  {
    sprintf(&devUniqueID[preSizeBytes + j], "%02X", macAddr[i]);
    j += 2;
  }
  Serial.println(devUniqueID);
}

#endif // defined(ESP32)