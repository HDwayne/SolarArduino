#include "Logger.h"

Logger Log;

Logger::Logger()
{
  webServer = nullptr;
}

void Logger::begin(unsigned long baudrate)
{
  Serial.begin(baudrate);
  while (!Serial)
    ;

#ifdef ESP32
  webServerModule.begin();
  webServer = &webServerModule;
#endif
}

size_t Logger::write(uint8_t byte)
{
  Serial.write(byte);

#ifdef ESP32
  if (webServer && WiFi.isConnected())
  {
    if (byte == '\n')
    {
      webServer->broadcastMessage(lineBuffer + '\n');
      lineBuffer = "";
    }
    else
    {
      lineBuffer += (char)byte;
    }
  }
#endif

  return 1;
}

size_t Logger::write(const uint8_t *buffer, size_t size)
{
  Serial.write(buffer, size);

#ifdef ESP32
  if (webServer && WiFi.isConnected())
  {
    for (size_t i = 0; i < size; i++)
    {
      uint8_t byte = buffer[i];
      if (byte == '\n')
      {
        webServer->broadcastMessage(lineBuffer + '\n');
        lineBuffer = "";
      }
      else
      {
        lineBuffer += (char)byte;
      }
    }
  }
#endif

  return size;
}
