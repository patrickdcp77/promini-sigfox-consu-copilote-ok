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

#define DHTPIN 11     // Pin de données connectée au capteur DHT22
#define DHTTYPE DHT22 // Définir le type de capteur DHT

DHT dht(DHTPIN, DHTTYPE);

const bool debug = true; // Définir à true pour activer le débogage, false pour désactiver
unsigned long lastSendTime = 0; // Variable pour stocker le dernier temps d'envoi
const unsigned long sendInterval = 3000; // Intervalle d'envoi en millisecondes (30 secondes)

void setup() {
  Serial.begin(9600); // Port série pour le moniteur série et le module Wisol
  pinMode(10, OUTPUT); // Définir la broche 10 comme sortie pour l'alimentation
  digitalWrite(10, HIGH); // Activer l'alimentation du capteur
  dht.begin();
  
  // Configurer la référence de tension interne à 1,1V
  analogReference(INTERNAL);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;

    // Désactiver l'ADC
    ADCSRA &= B01111111;

    // Activer l'ADC
    ADCSRA |= B10000000;

    // Effectuer une mesure pour stabiliser l'ADC
    int test = analogRead(A0);

    // Mesurer la tension de la batterie
    int sensorValue = analogRead(A0);
    if (debug) {
      Serial.print("Valeur brute du capteur: ");
      Serial.println(sensorValue);
    }

    // Convertir la valeur brute en tension avec référence 1,1V
    float voltage = sensorValue * (1.1 / 1023.0);
    float batteryVoltage = voltage * ((47.0 + 10.0) / 10.0); // Ajuster la tension en fonction du diviseur de tension
    uint16_t batteryVoltageMV = batteryVoltage * 1000; // Convertir la tension en millivolts

    if (debug) {
      Serial.print("Tension mesurée: ");
      Serial.println(voltage);
      Serial.print("Tension de la batterie (V): ");
      Serial.println(batteryVoltage);
      Serial.print("Tension de la batterie (mV): ");
      Serial.println(batteryVoltageMV);
    }

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

    // Créer la trame avec la tension, la température et l'humidité
    uint8_t frame[6];
    frame[0] = (batteryVoltageMV >> 8) & 0xFF;
    frame[1] = batteryVoltageMV & 0xFF;
    frame[2] = (temperature >> 8) & 0xFF;
    frame[3] = temperature & 0xFF;
    frame[4] = (humidity >> 8) & 0xFF;
    frame[5] = humidity & 0xFF;

    if (debug) {
      // Afficher la trame envoyée pour le débogage
      Serial.print("Trame envoyée: ");
      for (int i = 0; i < 6; i++) {
        Serial.print(frame[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    // Envoyer la trame via le module Wisol en utilisant des commandes AT
    Serial.print("AT$SF=");
    for (int i = 0; i < 6; i++) {
      if (frame[i] < 16) {
        Serial.print("0"); // Ajouter un zéro pour les valeurs inférieures à 16
      }
      Serial.print(frame[i], HEX);
    }
    Serial.println();
  }
}
