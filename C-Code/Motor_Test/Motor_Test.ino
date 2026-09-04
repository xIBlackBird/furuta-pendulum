// --- Pin-Definitionen für den BTS7960 ---
const int RPWM = 5; // Vorwärts
const int LPWM = 6; // Rückwärts
const int R_IS = A0; // Strommessung Vorwärts
const int L_IS = A1; // Strommessung Rückwärts

int aktuelleGeschwindigkeit = 0;
unsigned long letztesStromUpdate = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  // Analoge Pins brauchen kein pinMode, sie sind standardmäßig Eingänge

  motorStoppen();
  
  Serial.println("=== Motor-Testumgebung mit Strommessung ===");
  Serial.println("[w] : +10 PWM (Vorwärts)");
  Serial.println("[s] : -10 PWM (Rückwärts)");
  Serial.println("[x] : NOT-HALT (0 PWM)");
  Serial.println("-------------------------------------------");
}

void loop() {
  // 1. Befehle vom PC lesen
  if (Serial.available() > 0) {
    char befehl = Serial.read();
    
    if (befehl == 'w') aktuelleGeschwindigkeit += 10;
    else if (befehl == 's') aktuelleGeschwindigkeit -= 10;
    else if (befehl == 'x') aktuelleGeschwindigkeit = 0;

    aktuelleGeschwindigkeit = constrain(aktuelleGeschwindigkeit, -255, 255);
    motorAktualisieren(aktuelleGeschwindigkeit);

    if (befehl == 'w' || befehl == 's' || befehl == 'x') {
      Serial.print("Neues PWM-Signal: ");
      Serial.println(aktuelleGeschwindigkeit);
    }
  }

  // 2. Strom zyklisch messen und ausgeben (alle 500 ms)
  if (millis() - letztesStromUpdate > 500) {
    if (aktuelleGeschwindigkeit != 0) {
      messeUndDruckeStrom();
    }
    letztesStromUpdate = millis();
  }
}

// --- Hilfsfunktionen ---

void messeUndDruckeStrom() {
  int adc_rohwert = 0;
  
  // Je nach Richtung den richtigen Pin auslesen
  if (aktuelleGeschwindigkeit > 0) {
    adc_rohwert = analogRead(R_IS);
  } else if (aktuelleGeschwindigkeit < 0) {
    adc_rohwert = analogRead(L_IS);
  }

  // Umrechnung: ADC-Wert (0-1023) in Spannung (0-5V)
  float spannung = (adc_rohwert / 1023.0) * 5.0;
  
  // Umrechnung: Spannung in Ampere (grob geschätzt für Standard IBT-2)
  // (Bei manchen Platinen ist der Teiler leicht anders, aber als Richtwert perfekt)
  float strom_ampere = spannung / 1.17; 

  Serial.print("   -> Motorstrom: ");
  Serial.print(strom_ampere, 3); // Mit 3 Nachkommastellen
  Serial.print(" A  (Rohwert: ");
  Serial.print(adc_rohwert);
  Serial.println(")");
}

void motorAktualisieren(int pwmWert) {
  if (pwmWert > 0) {
    analogWrite(RPWM, pwmWert);
    analogWrite(LPWM, 0); 
  } 
  else if (pwmWert < 0) {
    analogWrite(LPWM, abs(pwmWert));
    analogWrite(RPWM, 0);
  } 
  else {
    motorStoppen();
  }
}

void motorStoppen() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}