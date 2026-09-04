// --- Pin-Definitionen ---
const int ENCODER_A = 2; 
const int ENCODER_B = 3; 

// Volatile Variable für den Hardware-Interrupt
volatile long encoderPosition = 0;

// Systemkonstante (64 CPR * 30er Getriebe)
const float TICKS_PRO_UMDREHUNG = 1920.0; 

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // 4X Decoding: Wir triggern bei JEDEM Flankenwechsel (CHANGE) an BEIDEN Pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoderB, CHANGE);
}

void loop() {
  static unsigned long letztesUpdate = 0;
  
  if (millis() - letztesUpdate > 50) { 
    // Interrupts kurz pausieren für sicheres Auslesen
    noInterrupts();
    long aktuellePosition = encoderPosition;
    interrupts();

    // --- Die Umrechnung in Radiant ---
    // (Position / Ticks pro Umdrehung) * 2 * PI
    float winkelRad = (aktuellePosition / TICKS_PRO_UMDREHUNG) * 2.0 * PI;

    // Ausgabe (mit 4 Nachkommastellen für die hohe Präzision)
    Serial.print("Ticks: ");
    Serial.print(aktuellePosition);
    Serial.print("  |  Zustand x1 (rad): ");
    Serial.println(winkelRad, 4); 
    
    letztesUpdate = millis();
  }
}

// --- Die Interrupt-Routinen für 4X Auswertung ---
// --- Die korrigierten Interrupt-Routinen für CCW = positiv ---
void updateEncoderA() {
  if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) {
    encoderPosition--; // Zählt jetzt runter
  } else {
    encoderPosition++; // Zählt jetzt hoch
  }
}

void updateEncoderB() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderPosition--; // Zählt jetzt runter
  } else {
    encoderPosition++; // Zählt jetzt hoch
  }
}