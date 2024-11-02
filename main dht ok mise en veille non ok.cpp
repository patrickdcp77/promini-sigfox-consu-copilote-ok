/*
dht ok
trame reçue ok
tension...
modifications sur PRO MINI:
régulateur enlevé
résistance enlevée de la led de visualisation de l'alimentation
mesure de la tension batterie appliquée sur VCC
on met une résistance CMS de 47K entre A0 et A1 coté supérieur du processeur
on met une résistance CMS de 10K entre A0 et le plan de masse en parallele à un condensateur de 1,1 nano soudés l'un sur l'autre


*/

#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#define DHTPIN 11     // Pin de données connectée au capteur DHT22
#define DHTTYPE DHT22 // Définir le type de capteur DHT

DHT dht(DHTPIN, DHTTYPE);

const bool debug = false; //true; // Définir à true pour activer le débogage, false pour désactiver
volatile bool shouldMeasure = false;

void setup() {
  Serial.begin(9600); // Port série pour le moniteur série et le module Wisol
  delay(100); // Attendre que le port série soit prêt
  pinMode(10, OUTPUT); // Définir la broche 10 comme sortie pour l'alimentation
  digitalWrite(10, HIGH); // Activer l'alimentation du capteur
  dht.begin();
  
  // Configurer le watchdog timer pour réveiller le processeur toutes les 8 secondes
  cli(); // Désactiver les interruptions
  MCUSR &= ~(1 << WDRF); // Effacer le flag de réinitialisation du watchdog
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Activer la possibilité de changer les paramètres du watchdog
  WDTCSR = (1 << WDP3) | (1 << WDP0); // Configurer le watchdog pour une période de 8 secondes
  WDTCSR |= (1 << WDIE); // Activer l'interruption du watchdog
  sei(); // Activer les interruptions
}

void loop() {
  if (shouldMeasure) {
    shouldMeasure = false;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      if (debug) {
        Serial.println("Échec de la lecture du capteur DHT!");
      }
      return;
    }

    if (debug) {
      Serial.print("Humidité: ");
      Serial.println(h);
      Serial.print("Température: ");
      Serial.println(t);
    }

    // Convertir les valeurs en entiers
    uint16_t humidity = (uint16_t)(h * 100);
    uint16_t temperature = (uint16_t)(t * 100);

    // Créer la trame avec la température et l'humidité
    uint8_t frame[4];
    frame[0] = (temperature >> 8) & 0xFF;
    frame[1] = temperature & 0xFF;
    frame[2] = (humidity >> 8) & 0xFF;
    frame[3] = humidity & 0xFF;

    if (debug) {
      // Afficher la trame envoyée pour le débogage
      Serial.print("Trame envoyée: ");
      for (int i = 0; i < 4; i++) {
        Serial.print(frame[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    // Envoyer la trame via le module Wisol en utilisant des commandes AT
    Serial.print("AT$SF=");
    for (int i = 0; i < 4; i++) {
      if (frame[i] < 16) {
        Serial.print("0"); // Ajouter un zéro pour les valeurs inférieures à 16
      }
      Serial.print(frame[i], HEX);
    }
    Serial.println();

    // Mettre le processeur en veille après les mesures
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
  }

  // Mettre le processeur en veille
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}

// Interruption du watchdog
ISR(WDT_vect) {
  static uint8_t count = 0;
  count++;
  if (count >= 8) { // 8 * 8 secondes = 64 secondes (environ 1 minute)
    count = 0;
    shouldMeasure = true;
  }
}
