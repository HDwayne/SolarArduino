#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "modules/WebServerModule.h"

class Logger : public Print
{
public:
  Logger();
  void begin(unsigned long baudrate = 9600);
  size_t write(uint8_t byte) override;
  size_t write(const uint8_t *buffer, size_t size) override;

private:
  WebServerModule *webServer;
  String lineBuffer;
};

#endif // LOGGER_H
