/*
test_avec_A1_niveau_haut et vcc mesuré à 4.14V
mesure correcte
*/

#include <Adafruit_Sensor.h>

// Résistances du pont diviseur
const float R1 = 47000.0;  // Résistance entre A0 et Vcc (47kΩ)
const float R2 = 10000.0;  // Résistance entre A0 et la masse (10kΩ)

void setup() {
  pinMode(A1, OUTPUT);       // Configurer A1 en sortie
  digitalWrite(A1, HIGH);    // Mettre A1 au niveau haut (5V)
  analogReference(INTERNAL); // Utiliser la référence interne de 1,1 V
  Serial.begin(9600);
}

void loop() {
  int valeurADC = analogRead(A0); // Lire la tension sur A0
  float tensionA0 = (valeurADC * 1.1) / 1023.0; // Tension sur A0 en volts
  float tensionSource = tensionA0 * ((R1 + R2) / R2); // Calcul de la tension d'entrée d'A1
  Serial.print("Tension d'A1 (estimée) : ");
  Serial.println(tensionSource);
  delay(5000);
}