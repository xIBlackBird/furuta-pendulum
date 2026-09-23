#include <Wire.h>
 
// --- LQR Regler Konstanten ---
const float K1 = -0.2127; 
const float K2 = 15.4930; 
const float K3 = -0.5594; 
const float K4 = 1.4194; 

// --- Pins ---
const int LPWM_PIN = 5; 
const int RPWM_PIN = 6; 
const int ENCODER_A = 2; 
const int ENCODER_B = 3; 
const int BUTTON_PIN = 4; // Start/Stop Knopf

// --- AS5600 (Pendel) ---
const int AS5600_ADDR = 0x36;
const int RAW_ANGLE_REG = 0x0C;
uint16_t letzterRohWertAS5600 = 0;
long kontinuierlichePositionAS5600 = 0; 
bool ersterAS5600Wert = true;

// --- Encoder (Motorarm) ---
volatile long encoderPosition = 0;

// --- Systemkonstanten ---
const float TICKS_PRO_UMDREHUNG_ARM = 1920.0;
const float TICKS_PRO_UMDREHUNG_PENDEL = 4096.0;

// --- Zustandsvektor & State-Machine ---
float x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
float x1_alt = 0.0, x2_alt = 0.0;
float last_phi2_dot = 0.0;
float x1_integral = 0.0;

// WICHTIG: Startet im Standby (6)
int controller_state = 6; 
unsigned long state_0_start = 0; 

// --- Timing ---
unsigned long letzterZeitpunkt = 0;
const unsigned long abtastZeit_us = 5000; 

void setup() {
  Serial.begin(500000); 
  Wire.begin();
  Wire.setClock(400000); 

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoderB, CHANGE);

  pinMode(LPWM_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Interner Pull-Up für den Knopf

  Serial.println("=== Standalone Pendel bereit (Warte auf Knopf) ===");
  letzterZeitpunkt = micros();
}

void loop() {
  // --- 1. Knopf abfragen (Start / Stop Toggle) ---
  bool aktuellerKnopfZustand = digitalRead(BUTTON_PIN);
  static bool letzterKnopfZustand = HIGH;

  // Wenn der Knopf gedrückt wird (zieht auf LOW)
  if (aktuellerKnopfZustand == LOW && letzterKnopfZustand == HIGH) {
    
    // Wenn das System im Stand-By ist, starte den Versuch
    if (controller_state == 6) {
      // Nullpunkt für den Motorarm genau hier setzen!
      noInterrupts();
      encoderPosition = 0; 
      interrupts();
      
      controller_state = 0;
      state_0_start = millis();
      letzterZeitpunkt = micros(); // Timer synchronisieren
    } 
    // Wenn das System läuft, stoppe es (Not-Aus per Knopf)
    else {
      controller_state = 5; // in Return to Home
      x1_integral = 0.0;
    }
  }
  letzterKnopfZustand = aktuellerKnopfZustand;


  // --- 2. Harte 5-ms Regelschleife ---
  unsigned long aktuellerZeitpunkt = micros();
  
  if (aktuellerZeitpunkt - letzterZeitpunkt >= abtastZeit_us) {
    float dt = (aktuellerZeitpunkt - letzterZeitpunkt) / 1000000.0;
    letzterZeitpunkt += abtastZeit_us; 

    // --- Sensoren auslesen ---
    noInterrupts();
    long aktuelleEncoderPos = encoderPosition;
    interrupts();

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    
    if (Wire.available() == 2) {
      uint16_t aktuellerRohWertAS5600 = (Wire.read() << 8) | Wire.read();
      if (ersterAS5600Wert) {
         letzterRohWertAS5600 = aktuellerRohWertAS5600;
         ersterAS5600Wert = false;
      }
      int16_t delta = aktuellerRohWertAS5600 - letzterRohWertAS5600;
      if (delta > 2048) delta -= 4096;  
      if (delta < -2048) delta += 4096; 
      kontinuierlichePositionAS5600 += delta;
      letzterRohWertAS5600 = aktuellerRohWertAS5600;
    }

    // --- Winkel & Geschwindigkeiten berechnen ---
    float x1_kontinuierlich = (aktuelleEncoderPos / TICKS_PRO_UMDREHUNG_ARM) * 2.0 * PI;
    float x2_kontinuierlich = (kontinuierlichePositionAS5600 / TICKS_PRO_UMDREHUNG_PENDEL) * 2.0 * PI;

    float x3_roh = (x1_kontinuierlich - x1_alt) / dt;
    float x4_roh = (x2_kontinuierlich - x2_alt) / dt;

    float alpha = 0.2; 
    x3 = (alpha * x3_roh) + ((1.0 - alpha) * x3);
    x4 = (alpha * x4_roh) + ((1.0 - alpha) * x4);

    x1_alt = x1_kontinuierlich;
    x2_alt = x2_kontinuierlich;

    // Wrapping
    x1 = x1_kontinuierlich;
    while (x1 > PI)  x1 -= 2.0 * PI;
    while (x1 < -PI) x1 += 2.0 * PI;

    x2 = x2_kontinuierlich - PI; 
    while (x2 > PI)  x2 -= 2.0 * PI;
    while (x2 < -PI) x2 += 2.0 * PI;


    // --- STATE MACHINE ---
    float u_volt = 0.0;
    float phi2_dot = x4;

    // A. Automatischer Not-Aus (175 Grad Limit)
    if (abs(x1_kontinuierlich) > (175.0 * PI / 180.0)) {
      controller_state = 6;
    }
    // B. LQR Fangbereich
    else if (abs(x2) < 0.35 && controller_state < 4) {
      controller_state = 4;
    }

    // C. Phasen
    switch (controller_state) {
      case 0:
        u_volt = 4.0;
        if (millis() - state_0_start > 100) controller_state = 1;
        break;
        
      case 1:
        u_volt = 0.0;
        if (phi2_dot * last_phi2_dot <= 0.0) controller_state = 2;
        break;
        
      case 2:
        if (x1 > -20.0 * PI / 180.0) u_volt = -6.0;
        else u_volt = 0.0;
        
        if (x1 < -60.0 * PI / 180.0) controller_state = 3;
        break;
        
      case 3:
        if (x1 < -25.0 * PI / 180.0) u_volt = 6.0;
        else u_volt = 0.0;
        
        if (phi2_dot * last_phi2_dot <= 0.0) controller_state = 1; 
        break;
        
      case 4: // LQR Aktiv
        u_volt = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
        break;

      case 5: // Return to Home
        {
          float Kp_home = 2.0; // P-Anteil: Zieht den Arm zur 0 (Ausprobieren!)
          float Kd_home = 0.5; // D-Anteil: Bremst den Arm ab, je schneller er wird
          float Ki_home = 0.2; // I-Anteil: Summiert die vergangenen Fehler auf

          if (abs(x1) > 0.05) {
            // 1. Integral aufsummieren (Fehler * Zeit)
            x1_integral += x1 * dt;
            
            // 2. Anti-Windup: Den I-Anteil so begrenzen, dass er maximal +/- 1.5 Volt zur Gesamtspannung beitragen kann
            float max_i_wirkung = 1.5;
            x1_integral = constrain(x1_integral, -max_i_wirkung / Ki_home, max_i_wirkung / Ki_home);
            
            // 3. PID-Gleichung
            u_volt = -(Kp_home * x1 + Kd_home * x3 + Ki_home * x1_integral);
            
            // 4. Gesamtspannung begrenzen (min 3V, damit er nicht beim Anfahren stottert)
            u_volt = constrain(u_volt, -3.0, 3.0); 
          } else {
            // Ziel erreicht! Motor aus und in den Standby wechseln
            controller_state = 6;
          }
        }
        break;
      case 6: // STANDBY, bzw Not-Aus
        u_volt = 0.0;
        break;
    }

    last_phi2_dot = phi2_dot;

    // --- PWM Ausgabe ---
    float pwm_min = 0.0;
    float totzone_volt = (pwm_min/255.0) * 12.0; 
    int pwm_output = 0;
    
    if (u_volt > 0.01) {
      pwm_output = constrain(((u_volt + totzone_volt) / 12.0) * 255, 0, 255);
      analogWrite(LPWM_PIN, 0);
      analogWrite(RPWM_PIN, pwm_output);
    } else if (u_volt < -0.01) {
      pwm_output = constrain(((abs(u_volt) + totzone_volt) / 12.0) * 255, 0, 255);
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, pwm_output);
    } else {
      analogWrite(LPWM_PIN, 0);
      analogWrite(RPWM_PIN, 0);
    }

    // Daten werden weiterhin gefunkt (falls ein PC verbunden ist)
    Serial.print(millis() / 1000.0, 3); Serial.print(",");
    Serial.print(controller_state);     Serial.print(",");
    Serial.print(x1, 4);                Serial.print(",");
    Serial.print(x2, 4);                Serial.print(",");
    Serial.print(x3, 4);                Serial.print(",");
    Serial.print(x4, 4);                Serial.print(",");
    Serial.println(u_volt, 3);
  }
}

// --- Interrupt-Routinen ---
void updateEncoderA() {
  if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) encoderPosition--;
  else encoderPosition++;
}
void updateEncoderB() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) encoderPosition--;
  else encoderPosition++;
}