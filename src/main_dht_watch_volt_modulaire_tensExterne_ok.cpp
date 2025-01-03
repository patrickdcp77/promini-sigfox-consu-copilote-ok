/*
dht ok+++

trame reçue ok

12345678 et mesures au début ok 

watchdog toute les 8s 112 fois puis mesures OK

les mesures sont arrondies à la partie entière ok

donc 1 octet suffirait pour chaque mesure

on garde les 2 octets pour l'instant

la tension de la batterie sur 2 octets car en mV ok

mesure tension via référence interne OK

essai avec mesure directe de A0 après avoir mis A1 à HIGH (Vcc qui doit être contrôlé)

trame envoyée 2 oct temp 2 oct hum 2 oct tension batterie  ok

mise en veille durant 1/4h par cycle

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
#define VOLTAGE_PIN A0 // Pin pour mesurer la tension au diviseur de tension
#define DHT_POWER_PIN 10 // Pin pour alimenter le capteur DHT22

DHT dht(DHTPIN, DHTTYPE);

const bool debug = false;//true; // Définir à true pour activer le débogage, false pour désactiver
volatile bool shouldMeasure = false;
volatile uint16_t wakeUpCounter = 0;
const uint16_t wakeUpLimit = 112; /// 112 * 8 secondes = 1/4h
int16_t humidity;
int16_t temperature;
uint16_t tensionSourceMV;
float tensionSource; // Déclarer tensionSource comme variable globale

// Résistances du pont diviseur
const float R1 = 47000.0;  // Résistance entre A0 et Vcc
const float R2 = 10000.0;  // Résistance entre A0 et la masse


// Prototypes des fonctions
void resetModule();
void measureDHT();
void measureA1Voltage();
void createFrame(uint8_t* frame);
void sendFrame(uint8_t* frame);
void setup();
void loop();
ISR(WDT_vect);


void resetModule() {
  digitalWrite(RESET_PIN, LOW);
  delay(100); // Attendre que le module soit réinitialisé
  digitalWrite(RESET_PIN, HIGH);
  delay(100); // Attendre que le module soit prêt
}

void measureDHT() {
  // Activer l'alimentation du capteur
  digitalWrite(DHT_POWER_PIN, HIGH);
  delay(2000); // Attendre que le capteur DHT se stabilise

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
  humidity = (int16_t)floor(h);
  temperature = (int16_t)floor(t);

  // Désactiver l'alimentation du capteur
  digitalWrite(DHT_POWER_PIN, LOW);
}

void measureA1Voltage() {

  pinMode(A1, OUTPUT);       // Configurer A1 en sortie
  digitalWrite(A1, HIGH);    // Mettre A1 au niveau haut (5V)
  analogReference(INTERNAL); // Utiliser la référence interne de 1,1 V
 

  // Activer l'ADC
  ADCSRA |= (1 << ADEN);
  // Lire la valeur de l'ADC sur la broche A1
  int valeurADC = analogRead(A0); // Lire la tension sur A0
   // Désactiver l'ADC
  ADCSRA &= ~(1 << ADEN);

  float tensionA0 = (valeurADC * 1.1) / 1023.0; // Tension sur A0 en volts
  float tensionSource = tensionA0 * ((R1 + R2) / R2); // Calcul de la tension d'entrée d'A1
  tensionSourceMV = (uint16_t)(tensionSource * 1000); // Convertir la tension en millivolts et en entier

  if (debug) {
    Serial.print("Valeur brute du capteur A1: ");
    Serial.println(tensionA0);
    Serial.print("Tension d'A1 (estimée) : ");
    Serial.println(tensionSourceMV);
  }
}

void createFrame(uint8_t* frame) {
  frame[0] = (temperature >> 8) & 0xFF;
  frame[1] = temperature & 0xFF;
  frame[2] = (humidity >> 8) & 0xFF;
  frame[3] = humidity & 0xFF;
  frame[4] = (tensionSourceMV >> 8) & 0xFF;
  frame[5] = tensionSourceMV & 0xFF;

  if (debug) {
    // Afficher la trame créée pour le débogage
    Serial.print("Trame créée: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(frame[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void sendFrame(uint8_t* frame) {
  // Envoyer la trame via le module Wisol en utilisant des commandes AT
  Serial.print("AT$SF=");
  for (int i = 0; i < 6; i++) {
    if (frame[i] < 16) {
      Serial.print("0"); // Ajouter un zéro pour les valeurs inférieures à 16
    }
    Serial.print(frame[i], HEX);
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

void setup() {
  Serial.begin(9600); // Port série pour le moniteur série et le module Wisol
  delay(100); // Attendre que le port série soit prêt
  pinMode(DHT_POWER_PIN, OUTPUT); // Définir la broche d'alimentation du capteur DHT22 comme sortie
  digitalWrite(DHT_POWER_PIN, LOW); // Désactiver l'alimentation du capteur au démarrage
  pinMode(RESET_PIN, OUTPUT); // Définir la broche de réinitialisation comme sortie

  if (debug) {
    Serial.print("Nom du fichier en cours: ");
    Serial.println(__FILE__);
  }

  // Réinitialiser le module Wisol
  resetModule();

  // Envoyer une commande AT pour vérifier la communication
  Serial.println("AT");
  delay(100); // Attendre que la commande soit envoyée

  dht.begin();

  // Envoyer une première trame de bon fonctionnement
  Serial.println("AT$SF=12345678");
  delay(100); // Attendre que la trame soit envoyée

  // Mesurer la température et l'humidité
  measureDHT();

  // Mesurer la tension sur A1
  measureA1Voltage();

  // Créer la trame
  uint8_t frame[6];
  createFrame(frame);

  // Envoyer la trame
  sendFrame(frame);

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

    // Réinitialiser le module Wisol au début de chaque boucle
    resetModule();

    // Envoyer une commande AT pour vérifier la communication
    Serial.println("AT");
    delay(100); // Attendre que la commande soit envoyée

    // Mesurer la température et l'humidité
    measureDHT();

    // Mesurer la tension sur A1
    measureA1Voltage();

    // Créer la trame
    uint8_t frame[6];
    createFrame(frame);

    // Envoyer la trame
    sendFrame(frame);

    // Remettre le processeur en veille
    if (debug) {
      Serial.println("Mise en veille...");
    }
  }

  // Mettre le processeur en veille
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}



