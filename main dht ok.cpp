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
const unsigned long sendInterval = 30000; // Intervalle d'envoi en millisecondes (30 secondes)

void setup() {
  Serial.begin(9600); // Port série pour le moniteur série et le module Wisol
  pinMode(10, OUTPUT); // Définir la broche 10 comme sortie pour l'alimentation
  digitalWrite(10, HIGH); // Activer l'alimentation du capteur
  dht.begin();

  // Envoyer une première trame de bon fonctionnement
  Serial.println("AT$SF=12345678");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      if (debug) {
        Serial.println("Échec de la lecture du capteur DHT!");
      }
      return;
    }

    // Convertir les valeurs en entiers
    uint16_t humidity = (uint16_t)(h * 100);
    uint16_t temperature = (uint16_t)(t * 100);

    // Créer la trame de 2 octets par mesure
    uint8_t frame[4];
    frame[0] = (humidity >> 8) & 0xFF;
    frame[1] = humidity & 0xFF;
    frame[2] = (temperature >> 8) & 0xFF;
    frame[3] = temperature & 0xFF;

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

    // Mesurer la tension de la batterie
    int sensorValue = analogRead(A0);
    float voltage = sensorValue * (5.0 / 1023.0);
    float batteryVoltage = voltage * ((47.0 + 10.0) / 10.0); // Calculer la tension de la batterie

    if (debug) {
      Serial.print("Tension de la batterie: ");
      Serial.print(batteryVoltage);
      Serial.println(" V");
    }
  }
}
