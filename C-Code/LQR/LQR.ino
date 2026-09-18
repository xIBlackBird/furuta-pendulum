#include <Wire.h>
 
// --- LQR Regler Konstanten (aus MATLAB) ---
const float K1 = -0.2127; // für x1 (Arm-Position)
const float K2 = 15.4930; // für x2 (Pendel-Position)
const float K3 = -0.5594; // für x3 (Arm-Geschwindigkeit)
const float K4 = 1.4194; // für x4 (Pendel-Geschwindigkeit)

// --- Motor-Pins (BTS7960) ---
const int LPWM_PIN = 5; // Pin für Linksherum
const int RPWM_PIN = 6; // Pin für Rechtsherum
/*
// hast du auf dem Board an Vcc verbunden
const int L_EN_PIN = 7; // Enable Links
const int R_EN_PIN = 8; // Enable Rechts
*/

// --- AS5600 (Pendel) ---
const int AS5600_ADDR = 0x36;
const int RAW_ANGLE_REG = 0x0C;
uint16_t letzterRohWertAS5600 = 0;
long kontinuierlichePositionAS5600 = 0; // Zählt über 360° hinaus weiter!
bool ersterAS5600Wert = true;

// --- Encoder (Motorarm) ---
const int ENCODER_A = 2; 
const int ENCODER_B = 3; 
volatile long encoderPosition = 0;

// --- Systemkonstanten ---
const float TICKS_PRO_UMDREHUNG_ARM = 1920.0;
const float TICKS_PRO_UMDREHUNG_PENDEL = 4096.0;

// --- Der Zustandsvektor ---
float x1 = 0.0; // Arm-Winkel (rad)
float x2 = 0.0; // Pendel-Winkel (rad)
float x3 = 0.0; // Arm-Geschwindigkeit (rad/s)
float x4 = 0.0; // Pendel-Geschwindigkeit (rad/s)

float x1_alt = 0.0;
float x2_alt = 0.0;

// --- Timing für die Regelschleife (Ts = 0.005s) ---
unsigned long letzterZeitpunkt = 0;
const unsigned long abtastZeit_us = 5000; // 5000 Mikrosekunden = 5ms

// Hilfsvariable für eine saubere Bildschirmausgabe
int printCounter = 0;

void setup() {
  Serial.begin(115200);
  
  // I2C Setup
  Wire.begin();
  Wire.setClock(400000); // 400 kHz für schnelles Auslesen

  // Encoder Setup
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoderB, CHANGE);

  pinMode(LPWM_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);

  Serial.println("=== Sensorfusion gestartet ===");
  delay(1000);
  
  letzterZeitpunkt = micros();
}

void loop() {
  unsigned long aktuellerZeitpunkt = micros();
  
  // 1. Harte Regelschleife: Code nur exakt alle 5ms ausführen
  if (aktuellerZeitpunkt - letzterZeitpunkt >= abtastZeit_us) {
    
    // Tatsächliches dt (in Sekunden) berechnen für maximale Präzision
    float dt = (aktuellerZeitpunkt - letzterZeitpunkt) / 1000000.0;
    letzterZeitpunkt = aktuellerZeitpunkt;

    // 2. Encoder auslesen (Arm)
    noInterrupts();
    long aktuelleEncoderPos = encoderPosition;
    interrupts();

    // 3. AS5600 auslesen (Pendel)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    
    uint16_t aktuellerRohWertAS5600 = 0;
    if (Wire.available() == 2) {
      aktuellerRohWertAS5600 = (Wire.read() << 8) | Wire.read();
      
      // Die Wrap-Around Logik (Verhindert Sprünge beim 360°-Übergang)
      if (ersterAS5600Wert) {
         letzterRohWertAS5600 = aktuellerRohWertAS5600;
         ersterAS5600Wert = false;
      }
      
      int16_t delta = aktuellerRohWertAS5600 - letzterRohWertAS5600;
      if (delta > 2048) delta -= 4096;  // Überlauf vorwärts
      if (delta < -2048) delta += 4096; // Überlauf rückwärts
      
      kontinuierlichePositionAS5600 += delta;
      letzterRohWertAS5600 = aktuellerRohWertAS5600;
    }

    // 4. KONTINUIERLICHE Winkel in Radiant berechnen (Wichtig für die Ableitung!)
    float x1_kontinuierlich = (aktuelleEncoderPos / TICKS_PRO_UMDREHUNG_ARM) * 2.0 * PI;
    float x2_kontinuierlich = (kontinuierlichePositionAS5600 / TICKS_PRO_UMDREHUNG_PENDEL) * 2.0 * PI;

    // 5. x3 und x4 (Winkelgeschwindigkeiten) aus den KONTINUIERLICHEN Werten berechnen
    float x3_roh = (x1_kontinuierlich - x1_alt) / dt;
    float x4_roh = (x2_kontinuierlich - x2_alt) / dt;

    // --- NEU: EMA-Filter für ruhigere Geschwindigkeiten ---
    // alpha = 0.5 bedeutet: 50% der neuen Messung, 50% der alten Messung.
    // Mach alpha kleiner (z.B. 0.3), wenn es immer noch zittert.
    float alpha = 0.2; 
    x3 = (alpha * x3_roh) + ((1.0 - alpha) * x3);
    x4 = (alpha * x4_roh) + ((1.0 - alpha) * x4);

    // Werte für die NÄCHSTE Ableitung merken
    x1_alt = x1_kontinuierlich;
    x2_alt = x2_kontinuierlich;

    // 6. Wrapping der Winkel für den LQR-Regler auf den Bereich [-PI, +PI]
    x1 = x1_kontinuierlich;
    while (x1 > PI)  x1 -= 2.0 * PI;
    while (x1 < -PI) x1 += 2.0 * PI;

    x2 = x2_kontinuierlich;
    // Hier ziehen wir direkt den Offset ab, falls das Pendel beim Arduino-Start unten hing!
    // (Aus Unten=0 wird Unten=-PI. Aus Oben=PI wird Oben=0)
    x2 = x2 - PI; 
    
    // Wrapping für das Pendel
    while (x2 > PI)  x2 -= 2.0 * PI;
    while (x2 < -PI) x2 += 2.0 * PI;

    // --- 7. LQR Regler berechnen ---
    // u ist hier die benötigte SPANNUNG in Volt!
    float u_volt = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);

    // --- 8. Sicherheitsabschaltung ---
    // Wenn das Pendel weiter als z.B. 20° (0.35 rad) fällt, schalten wir den Motor ab, 
    // damit das System auf dem Tisch nicht völlig eskaliert. Nur für LQR, sonst swingup
    if (abs(x2) > 0.35) {
      u_volt = 0;
    }
    
    // --- 9. Totzonen-Kompensation und PWM-Umrechnung ---
    int pwm_output = 0;
    
    // Finde heraus, bei wie viel Volt dein Motor im echten Leben GANZ KNAPP anfängt zu drehen.
    // Beispiel: Er braucht mindestens 1.5V, um die innere Reibung zu überwinden.
    float pwm_min = 0.0;
    float totzone_volt = (pwm_min/255.0) * 12.0; 
    
    if (u_volt > 0.01) {
      // Motor soll vorwärts drehen
      float u_kompensiert = u_volt + totzone_volt; 
      pwm_output = (u_kompensiert / 12.0) * 255;
      pwm_output = constrain(pwm_output, 0, 255); // Sicherheit: Maximal 255
      
      analogWrite(LPWM_PIN, 0);
      analogWrite(RPWM_PIN, pwm_output);
      
    } else if (u_volt < -0.01) {
      // Motor soll rückwärts drehen
      float u_kompensiert = abs(u_volt) + totzone_volt;
      pwm_output = (u_kompensiert / 12.0) * 255;
      pwm_output = constrain(pwm_output, 0, 255);
      
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, pwm_output);
      
    } else {
      // Motor stoppen
      analogWrite(LPWM_PIN, 0);
      analogWrite(RPWM_PIN, 0);
    }
    // --- 10. Serielle Ausgabe (gedrosselt auf 20 Hz) ---
    printCounter++;
    if (printCounter >= 10) {
      Serial.print("x1(Arm): ");     Serial.print(x1, 2);
      Serial.print("\t x2(Pendel): "); Serial.print(x2, 2);
      Serial.print("\t x3(Arm_v): ");  Serial.print(x3, 2);
      Serial.print("\t x4(Pend_v): "); Serial.print(x4, 2);
      
      // Das berechnete Stellsignal (Volt)
      Serial.print("\t u(Volt): ");    Serial.print(u_volt, 2);
      
      // Das finale PWM-Signal (+ für rechts, - für links für die Anzeige)
      Serial.print("\t PWM: ");        
      if (u_volt < 0) {
        Serial.println(-pwm_output); // Zeigt im Plotter schön ins Negative
      } else {
        Serial.println(pwm_output);
      }
      
      printCounter = 0;
    }
  }
}

// --- Interrupt-Routinen für den Motor-Encoder ---
void updateEncoderA() {
  if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) encoderPosition--;
  else encoderPosition++;
}

void updateEncoderB() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) encoderPosition--;
  else encoderPosition++;
}