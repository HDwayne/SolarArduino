#include "RTCModule.h"

RTCModule rtcModule;

// ----------------- RTC Module Constructor -----------------

RTCModule::RTCModule() {}

// ----------------- RTC public functions -----------------

void RTCModule::init()
{
  if (!rtc.begin())
  {
    Serial.println("[ERROR] RTC module not found!");
    while (1)
      ; // Stop the program
  }
}

void RTCModule::adjustUTC(const char *date, const char *time)
{
  DateTime dateHeureUTC(date, time);
  rtc.adjust(dateHeureUTC);
}

void RTCModule::adjustLocal(const char *date, const char *time)
{
  DateTime dateHeureUTC(date, time);
  int nbreDheuresAretrancherOuAjouter = estOnEnHeureDeEte(dateHeureUTC) ? nombreDheuresArajouterOuEnleverEnEte : nombreDheuresArajouterOuEnleverEnHiver;
  DateTime dateHeureLocale(dateHeureUTC + TimeSpan(0, nbreDheuresAretrancherOuAjouter, 0, 0));
  rtc.adjust(dateHeureLocale);
}

DateTime RTCModule::getDateTimeUTC()
{
  return rtc.now();
}

DateTime RTCModule::getDateTimeLocal()
{
  DateTime dateHeureDuDS3231 = rtc.now();
  int nbreDheuresAajouterOuRetirer = estOnEnHeureDeEte(dateHeureDuDS3231) ? nombreDheuresArajouterOuEnleverEnEte : nombreDheuresArajouterOuEnleverEnHiver;
  return dateHeureDuDS3231 + TimeSpan(0, nbreDheuresAajouterOuRetirer, 0, 0);
}

// ----------------- RTC private functions -----------------

void RTCModule::print(DateTime dt)
{
  Serial.print(joursDeLaSemaine[dt.dayOfTheWeek()]);
  Serial.print(" ");
  Serial.print(dt.day());
  Serial.print(" ");
  Serial.print(moisDeLannee[dt.month() - 1]);
  Serial.print(" ");
  Serial.print(dt.year());
  Serial.print(" ");
  Serial.print(dt.hour());
  Serial.print(":");
  Serial.print(dt.minute());
  Serial.print(":");
  Serial.print(dt.second());
}

bool RTCModule::estOnEnHeureDeEte(DateTime dateHeureAanalyser)
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