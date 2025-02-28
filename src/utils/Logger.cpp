#include "Logger.h"

extern WebServerModule webServerModule;

Logger::Logger()
{
  webServer = nullptr;
}

void Logger::begin(unsigned long baudrate)
{
  Serial.begin(baudrate);
  while (!Serial)
    ;

  webServerModule.begin();
  webServer = &webServerModule;
}

size_t Logger::write(uint8_t byte)
{
  Serial.write(byte);

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

  return 1;
}

size_t Logger::write(const uint8_t *buffer, size_t size)
{
  Serial.write(buffer, size);

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

  return size;
}
