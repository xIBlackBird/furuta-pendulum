#include <Wire.h>

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
    x3 = (x1_kontinuierlich - x1_alt) / dt;
    x4 = (x2_kontinuierlich - x2_alt) / dt;

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

    // 7. Serielle Ausgabe
    // Wir geben nur jeden 10. Durchlauf (alle 50ms) aus, 
    // um die 5ms-Schleife nicht durch lange Textausgaben zu bremsen.
    printCounter++;
    if (printCounter >= 10) {
      Serial.print("x1: "); Serial.print(x1, 2);
      Serial.print(" | x2: "); Serial.print(x2, 2);
      Serial.print(" | x3 (rad/s): "); Serial.print(x3, 2);
      Serial.print(" | x4 (rad/s): "); Serial.println(x4, 2);
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