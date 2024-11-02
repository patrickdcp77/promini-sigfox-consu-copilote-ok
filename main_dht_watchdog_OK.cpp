/*
dht ok
trame reçue ok
12345678 et mesures au début 
watchdog toute les 8s 112 fois puis mesures OK
les mesures sont arrondies à la partie entière
donc 1 octet suffirait pour chaque mesure
on garde les 2 octets pour l'instant
reste à voir la tension de la batterie

*/

#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <math.h> // Inclure la bibliothèque mathématique pour utiliser floor()

#define DHTPIN 11     // Pin de données connectée au capteur DHT22
#define DHTTYPE DHT22 // Définir le type de capteur DHT
#define RESET_PIN 2   // Pin de réinitialisation du module Wisol

DHT dht(DHTPIN, DHTTYPE);

const bool debug = false;//true; // Définir à true pour activer le débogage, false pour désactiver
volatile bool shouldMeasure = false;
volatile uint16_t wakeUpCounter = 0;
const uint16_t wakeUpLimit = 112 ;//* 8 secondes = 896 secondes (environ 15 minutes)

void resetModule() {
  digitalWrite(RESET_PIN, LOW);
  delay(100); // Attendre que le module soit réinitialisé
  digitalWrite(RESET_PIN, HIGH);
  delay(100); // Attendre que le module soit prêt
}

void setup() {
  Serial.begin(9600); // Port série pour le moniteur série et le module Wisol
  delay(100); // Attendre que le port série soit prêt
  pinMode(10, OUTPUT); // Définir la broche 10 comme sortie pour l'alimentation
  digitalWrite(10, HIGH); // Activer l'alimentation du capteur
  pinMode(RESET_PIN, OUTPUT); // Définir la broche de réinitialisation comme sortie

  // Réinitialiser le module Wisol
  resetModule();

  // Envoyer une commande AT pour vérifier la communication
  Serial.println("AT");
  delay(100); // Attendre que la commande soit envoyée

  dht.begin();
  delay(2000); // Attendre que le capteur DHT se stabilise

  // Envoyer une première trame de bon fonctionnement
  Serial.println("AT$SF=12345678");
  delay(100); // Attendre que la trame soit envoyée

  // Lire les valeurs de température et d'humidité
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    if (debug) {
      Serial.println("Échec de la lecture du capteur DHT!");
    }
  } else {
    if (debug) {
      Serial.print("Humidité: ");
      Serial.println(h);
      Serial.print("Température: ");
      Serial.println(t);
    }

    // Prendre seulement la partie entière des valeurs
    int16_t humidity = (int16_t)floor(h);
    int16_t temperature = (int16_t)floor(t);

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
    delay(100); // Attendre que la trame soit envoyée
  }

  // Configurer le watchdog timer pour réveiller le processeur toutes les 8 secondes
  cli(); // Désactiver les interruptions
  MCUSR &= ~(1 << WDRF); // Effacer le flag de réinitialisation du watchdog
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Activer la possibilité de changer les paramètres du watchdog
  WDTCSR = (1 << WDP3) | (1 << WDP0); // Configurer le watchdog pour une période de 8 secondes
  WDTCSR |= (1 << WDIE); // Activer l'interruption du watchdog
  sei(); // Activer les interruptions

  // Mettre le processeur en veille
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

void loop() {
  if (shouldMeasure) {
    shouldMeasure = false;

    // Réinitialiser le module Wisol au début de chaque boucle
    resetModule();

    // Envoyer une commande AT pour vérifier la communication
    Serial.println("AT");
    delay(100); // Attendre que la commande soit envoyée

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

    // Prendre seulement la partie entière des valeurs
    int16_t humidity = (int16_t)floor(h);
    int16_t temperature = (int16_t)floor(t);

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
    delay(100); // Attendre que la trame soit envoyée

    // Remettre le processeur en veille
    if (debug) {
      Serial.println("Mise en veille...");
    }
    sleep_mode();
  }
}

// Interruption du watchdog
ISR(WDT_vect) {
  wakeUpCounter++;
  if (wakeUpCounter >= wakeUpLimit) {
    wakeUpCounter = 0;
    shouldMeasure = true;
  }
}
