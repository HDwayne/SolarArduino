#include "RTCModule.h"
#include "ConfigModule.h"

RTCModule rtcModule;

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

#if defined(ESP32)
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
#endif // ESP32

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
  // https://github.com/JeromeTGH/Programmes-autour-du-DS-3231/blob/main/prgArduino-3-ExempleAvecHeureEteHiver/prgArduino-3-ExempleAvecHeureEteHiver.ino?pseSrc=pgTutoDs3231
  // THIS FUNCTION IS IN FRENCH, AS IT IS SPECIFIC TO THE FRENCH TIMEZONE

  /*
   * En France, le passage à l'heure d'hiver s'effectue le dernier dimanche d'octobre,
   * et le passage à l'heure d'été, le dernier dimanche de mars.
   *
   * À la fin mars, on ajoute 1 heure, dans la nuit de samedi à dimanche.
   * Le changement se fait à 2 heures du matin, où il sera alors 3h du matin.
   *
   * À la fin d'octobre, on retire 1 heure, dans la nuit de samedi à dimanche.
   * Le changement se fait à 3 heures du matin, où il sera alors 2h du matin.
   *
   * À noter que ce changement d'heure va possiblement disparaître, dans les années à venir.
   * Mais là pour l'instant, en 2023, il est toujours "en cours".
   *
   */

  // Variables temporaires de cette fonction
  boolean estAvantLeDernierDimancheDeMars;
  boolean estApresLeDernierDimancheDeOctobre;

  // Récupération temporaire des données dont nous aurons besoin
  int anneeDeLaDateHeureAanalyser = dateHeureAanalyser.year();
  int moisDeLaDateHeureAanalyser = dateHeureAanalyser.month();
  int jourDeLaDateHeureAanalyser = dateHeureAanalyser.day();
  int heuresDeLaDateHeureAanalyser = dateHeureAanalyser.hour();
  int minutesDeLaDateHeureAanalyser = dateHeureAanalyser.minute();

  // Recherche du "numéro" de jour correspondant au dernier dimanche du mois de mars (de l'année de la date à analyser)
  int dernierDimancheDuMoisDeMars;
  for (int i = 31; i >= 1; i--)
  {
    DateTime jourDeMars(anneeDeLaDateHeureAanalyser, 3, i, 0, 0, 0); // Paramètres = année, mois, jour, heures, minutes, secondes
    if (jourDeMars.dayOfTheWeek() == 0)
    {
      dernierDimancheDuMoisDeMars = i; // "dayOfTheWeek == 0" signifiant que nous sommes un "dimanche", avec la librairie RTClib,
      break;                           // et le "break" permet alors de quitter cette boucle for, comme nous avons trouvé le dernier dimanche du mois
    }
  }

  // Recherche du "numéro" de jour correspondant au dernier dimanche du mois d'octobre (de l'année de la date à analyser)
  int dernierDimancherDuMoisDeOctobre;
  for (int i = 31; i >= 1; i--)
  {
    DateTime jourDeOctobre(anneeDeLaDateHeureAanalyser, 10, i, 0, 0, 0); // Paramètres = année, mois, jour, heures, minutes, secondes
    if (jourDeOctobre.dayOfTheWeek() == 0)
    {
      dernierDimancherDuMoisDeOctobre = i; // "dayOfTheWeek == 0" signifiant que nous sommes un "dimanche", avec la librairie RTClib,
      break;                               // et le "break" permet alors de quitter cette boucle for, comme nous avons trouvé le dernier dimanche du mois
    }
  }

  // On teste pour savoir si on est avant le dernier dimanche de mars, et avant 3h du matin
  if ((moisDeLaDateHeureAanalyser <= 3) && (jourDeLaDateHeureAanalyser <= dernierDimancheDuMoisDeMars) && (heuresDeLaDateHeureAanalyser < 3))
    estAvantLeDernierDimancheDeMars = true;
  else
    estAvantLeDernierDimancheDeMars = false;

  // On teste pour savoir si on est après le dernier dimanche d'octobre, et après 3h du matin
  // (remarque : surtout ne pas faire l'upload entre 2 et 3h du mat, ce jour là en particulier, sinon ça va dysfonctionner !)
  if ((moisDeLaDateHeureAanalyser >= 10) && (jourDeLaDateHeureAanalyser >= dernierDimancherDuMoisDeOctobre) && (heuresDeLaDateHeureAanalyser >= 3))
    estApresLeDernierDimancheDeOctobre = true;
  else
    estApresLeDernierDimancheDeOctobre = false;

  // Et on retourne le résultat
  if (estAvantLeDernierDimancheDeMars || estApresLeDernierDimancheDeOctobre)
    return false; // Car là, on serait en "heure d'hiver"
  else
    return true; // Car là, on serait en "heure d'été"
}