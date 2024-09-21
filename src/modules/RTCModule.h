// MODULE_RTC_H

#ifndef MODULE_RTC_H
#define MODULE_RTC_H

#include <Arduino.h>
#include <RTClib.h>

class RTCModule
{
public:
  RTCModule();

  void init();
  void adjust(const char *date, const char *time);
  void adjustUTC(const char *date, const char *time);
  void adjustLocal(const char *date, const char *time);
  DateTime getDateTimeUTC();
  DateTime getDateTimeLocal();

private:
  RTC_DS3231 rtc;

  const int nombreDheuresArajouterOuEnleverEnHiver = 1;
  const int nombreDheuresArajouterOuEnleverEnEte = 2;

  const char joursDeLaSemaine[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  const char moisDeLannee[12][12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

  void print(DateTime dateHeure);
  bool estOnEnHeureDeEte(DateTime dateHeure);
};

extern RTCModule rtcModule;

#endif // MODULE_RTC_H