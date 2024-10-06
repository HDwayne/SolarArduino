// MODULE_RTC_H

#ifndef MODULE_RTC_H
#define MODULE_RTC_H

#include <Arduino.h>
#include <RTClib.h>
#include "time.h"

#if defined(ESP32)
#include "WiFi.h"
#endif // ESP32

class RTCModule
{
public:
  RTCModule();

  void init();
  void adjustFromUTC(const char *date, const char *time);
  void adjustFromLocal(const char *date, const char *time);
#if defined(ESP32)
  void adjustFromNTP();
#endif // ESP32
  DateTime getDateTimeUTC();
  DateTime getDateTimeLocal();
  bool lostPower() { return rtc.lostPower(); }
  void printDateTime(const DateTime &dateHeure);

private:
  RTC_DS3231 rtc;

  const int DeltaHoursWinter = 1;
  const int DeltaHoursSummer = 2;

  const char dayOfWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  const char mounthOfYear[12][12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

  void info();
  bool estOnEnHeureDeEte(const DateTime &dateHeure); // French Winter/Summer time
};

extern RTCModule rtcModule;

#endif // MODULE_RTC_H