#include <Wire.h>
 
// --- LQR Regler Konstanten ---
const float K1 = -0.2127; 
const float K2 = 15.4930; 
const float K3 = -0.5594; 
const float K4 = 1.4194; 

// --- Motor-Pins (BTS7960) ---
const int LPWM_PIN = 5; 
const int RPWM_PIN = 6; 

// --- AS5600 (Pendel) ---
const int AS5600_ADDR = 0x36;
const int RAW_ANGLE_REG = 0x0C;
uint16_t letzterRohWertAS5600 = 0;
long kontinuierlichePositionAS5600 = 0; 
bool ersterAS5600Wert = true;

// --- Encoder (Motorarm) ---
const int ENCODER_A = 2; 
const int ENCODER_B = 3; 
volatile long encoderPosition = 0;

// --- Systemkonstanten ---
const float TICKS_PRO_UMDREHUNG_ARM = 1920.0;
const float TICKS_PRO_UMDREHUNG_PENDEL = 4096.0;

// --- Zustandsvektor & State-Machine ---
float x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
float x1_alt = 0.0, x2_alt = 0.0;
float last_phi2_dot = 0.0;

int controller_state = 0; 
unsigned long state_0_start = 0; // Timer für den ersten Schubser
bool is_running = false;

// --- Timing ---
unsigned long letzterZeitpunkt = 0;
const unsigned long abtastZeit_us = 5000; 

void setup() {
  Serial.begin(500000); // Maximale Baudrate für Echtzeit-Datenlogging!
  
  Wire.begin();
  Wire.setClock(400000); 

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoderB, CHANGE);

  pinMode(LPWM_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);

  // Warten auf Start-Signal vom Python-Skript
  while (!is_running) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'S') { // 'S' für Start
        is_running = true;
      }
    }
  }
  
  letzterZeitpunkt = micros();
  state_0_start = millis();
}

void loop() {
  unsigned long aktuellerZeitpunkt = micros();
  
  if (aktuellerZeitpunkt - letzterZeitpunkt >= abtastZeit_us) {
    float dt = (aktuellerZeitpunkt - letzterZeitpunkt) / 1000000.0;
    letzterZeitpunkt += abtastZeit_us; // Anti-Drift Timer

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

    x2 = x2_kontinuierlich - PI; // Null ist jetzt exakt oben!
    while (x2 > PI)  x2 -= 2.0 * PI;
    while (x2 < -PI) x2 += 2.0 * PI;

    // --- STATE MACHINE (Swing-Up & LQR) ---
    float u_volt = 0.0;
    float phi2_dot = x4;
    
    // 0. NOT-AUS (Allerhöchste Priorität!)
    // Wenn der Arm sich weiter als +/- 135 Grad vom Startpunkt entfernt
    if (abs(x1_kontinuierlich) > (135 * PI / 180.0)) {
      controller_state = 5;
    }

    // 1. LQR Fangbereich (Nur zulassen, wenn kein Not-Aus aktiv ist)
    else if (abs(x2) < 0.35 && controller_state != 5) {
      controller_state = 4;
    }

    // 2. Phasen-Logik
    switch (controller_state) {
      case 0:
        u_volt = 4.0;
        if (millis() - state_0_start > 100) { 
          controller_state = 1;
        }
        break;
        
      case 1:
        u_volt = 0.0;
        if (phi2_dot * last_phi2_dot <= 0.0) {
          controller_state = 2;
        }
        break;
        
      case 2:
        if (x1 > -20.0 * PI / 180.0) u_volt = -6.0; // von -10 auf -20
        else u_volt = 0.0;
        
        if (x1 < -60.0 * PI / 180.0) controller_state = 3;
        break;
        
      case 3:
        if (x1 < -25.0 * PI / 180.0) u_volt = 6.0;
        else u_volt = 0.0;
        
        /*
        // Sequenz darf nur abgefahren werden, wenn Pendel in der ausgangslage
        // Wenn das Pendel oben seinen Schwung verliert, neu ansetzen
        if (phi2_dot * last_phi2_dot <= 0.0) {
          controller_state = 1; 
        }
        */
        break;
        
      case 4: // LQR
        u_volt = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
        break;

      case 5: // NOT-AUS
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

    // --- Serieller CSV-Datenstrom ---
    // Format: Zeit,State,x1,x2,x3,x4,u_volt
    Serial.print(millis() / 1000.0, 3); Serial.print(",");
    Serial.print(controller_state);     Serial.print(",");
    Serial.print(x1, 4);                Serial.print(",");
    Serial.print(x2, 4);                Serial.print(",");
    Serial.print(x3, 4);                Serial.print(",");
    Serial.print(x4, 4);                Serial.print(",");
    Serial.println(u_volt, 3);
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