#include "RTCModule.h"

#include "WiFi.h"
#include "utils/Logger.h"
#include "modules/ConfigModule.h"

extern ConfigModule configModule;
extern Logger Log;

// ----------------- RTC Module Constructor -----------------

RTCModule::RTCModule() {}

// ----------------- RTC public functions -----------------

void RTCModule::init()
{
  if (!rtc.begin())
  {
    Log.println("[ERROR] RTC module not found!");
    while (1)
      ; // Stop the program
  }

  if (rtc.lostPower())
  {
    Log.println("[RTCModule] RTC lost power, adjusting from NTP server");
    adjustFromNTP();
  }

  info();
}

void RTCModule::adjustFromUTC(const char *date, const char *time)
{
  DateTime DateTimeUTC(date, time);
  rtc.adjust(DateTimeUTC);

  info();
}

void RTCModule::adjustFromLocal(const char *date, const char *time)
{
  DateTime DateTimeLocal(date, time);
  int delta = estOnEnHeureDeEte(DateTimeLocal) ? DeltaHoursSummer : DeltaHoursWinter;
  DateTime DateTimeUTC = DateTimeLocal - TimeSpan(0, delta, 0, 0);
  rtc.adjust(DateTimeUTC);

  info();
}

void RTCModule::adjustFromNTP()
{
  while (WiFi.status() != WL_CONNECTED)
  {
    Log.println("[RTCModule - NTP] WiFi not connected!");
    delay(1000);
  }

  configTime(0, 0, configModule.getNTPServer1(), configModule.getNTPServer2(), configModule.getNTPServer3());

  struct tm timeinfo;
  while (!getLocalTime(&timeinfo))
  {
    Log.println("[RTCModule - NTP] Failed to obtain time");
    delay(1000);
  }

  DateTime DateTimeUTC(
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec);

  rtc.adjust(DateTimeUTC);
  Log.println("[RTCModule - NTP] RTC adjusted from NTP server");
  info();
}

DateTime RTCModule::getDateTimeUTC()
{
  return rtc.now();
}

DateTime RTCModule::getDateTimeLocal()
{
  DateTime DateTimeUTC = rtc.now();
  int delta = estOnEnHeureDeEte(DateTimeUTC) ? DeltaHoursSummer : DeltaHoursWinter;
  return DateTimeUTC + TimeSpan(0, delta, 0, 0);
}

void RTCModule::printDateTime(const DateTime &dt)
{
  Log.print(dayOfWeek[dt.dayOfTheWeek() - 1]);
  Log.print(" ");
  Log.print(dt.day());
  Log.print(" ");
  Log.print(mounthOfYear[dt.month() - 1]);
  Log.print(" ");
  Log.print(dt.year());
  Log.print(" ");
  Log.print(dt.hour());
  Log.print(":");
  Log.print(dt.minute());
  Log.print(":");
  Log.print(dt.second());
}

// ----------------- RTC private functions -----------------

void RTCModule::info()
{
  Log.println(F("settings for the RTC module :"));
  Log.print(F("   → Delta time in winter : "));
  if (DeltaHoursWinter > 0)
    Log.print(F("+"));
  Log.print(DeltaHoursWinter);
  Log.print(F(" H"));
  Log.println();
  Log.print(F("   → Delta time in summer : "));
  if (DeltaHoursSummer > 0)
    Log.print(F("+"));
  Log.print(DeltaHoursSummer);
  Log.print(F(" H"));
  Log.println();
  Log.println();

  Log.println(F("Date/Hour (Local) : "));
  Log.print(F("   → "));
  printDateTime(getDateTimeLocal());
  Log.println();

  Log.println(F("Date/Hour (UTC) : "));
  Log.print(F("   → "));
  printDateTime(getDateTimeUTC());
  Log.println();
}

bool RTCModule::estOnEnHeureDeEte(const DateTime &dateHeureAanalyser)
{
  /*
   * En France :
   * - Passage à l'heure d'été : Dernier dimanche de mars, à 2h UTC (3h locale).
   * - Passage à l'heure d'hiver : Dernier dimanche d'octobre, à 2h UTC (3h locale).
   * - Les calculs sont basés sur UTC pour éviter les confusions locales.
   */

  int year = dateHeureAanalyser.year();
  int month = dateHeureAanalyser.month();
  int day = dateHeureAanalyser.day();
  int hour = dateHeureAanalyser.hour();
  int minute = dateHeureAanalyser.minute();

  // Find the last Sunday of March
  int lastSundayMarch = 31;
  while (DateTime(year, 3, lastSundayMarch, 0, 0, 0).dayOfTheWeek() != 0)
  {
    lastSundayMarch--;
  }

  // Find the last Sunday of October
  int lastSundayOctober = 31;
  while (DateTime(year, 10, lastSundayOctober, 0, 0, 0).dayOfTheWeek() != 0)
  {
    lastSundayOctober--;
  }

  // Check if before the transition to summer time
  if (month < 3 || (month == 3 && (day < lastSundayMarch || (day == lastSundayMarch && hour < 2))))
  {
    return false; // Winter time
  }

  // Check if after the transition to winter time
  if (month > 10 || (month == 10 && (day > lastSundayOctober || (day == lastSundayOctober && hour >= 2))))
  {
    return false; // Winter time
  }

  // If not in the above conditions, it's summer time
  return true;
}
