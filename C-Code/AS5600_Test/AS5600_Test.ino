#include <Wire.h> // Die Standard-Bibliothek für I2C

const int AS5600_ADDR = 0x36;    // I2C-Adresse des Sensors
const int RAW_ANGLE_REG = 0x0C;  // Register für den rohen Winkel (12 Bit)

void setup() {
  Serial.begin(115200);
  
  // I2C-Bus starten
  Wire.begin();
  // Bus-Geschwindigkeit auf 400 kHz setzen (wichtig für schnelle Regler!)
  Wire.setClock(400000); 

  Serial.println("=== AS5600 Winkelsensor Test ===");
  Serial.println("Warte auf Sensordaten...");
  delay(1000);
}

void loop() {
  // 1. Dem Sensor sagen, welches Register wir lesen wollen
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_REG);
  Wire.endTransmission(false); // Bus für die direkte Antwort offen halten

  // 2. Zwei Bytes (High und Low) vom Sensor anfordern
  Wire.requestFrom(AS5600_ADDR, 2);

  // 3. Wenn die 2 Bytes da sind, zusammensetzen
  if (Wire.available() == 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte  = Wire.read();
    
    // Die Bits zu einer Zahl zwischen 0 und 4095 verschmelzen
    uint16_t rawAngle = (highByte << 8) | lowByte;

    // --- Umrechnung für deinen LQR-Regler ---
    // In Grad (0° bis 360°)
    float winkelGrad = (rawAngle / 4095.0) * 360.0;
    
    // In Radiant (0 bis 2*PI) -> Das wird später dein Zustand x2!
    float winkelRad = (rawAngle / 4095.0) * 2.0 * PI;

    // Ausgabe in den Seriellen Monitor
    Serial.print("Rohwert: ");
    Serial.print(rawAngle);
    Serial.print("\t| Grad: ");
    Serial.print(winkelGrad, 1);
    Serial.print("° \t| Zustand x2 (rad): ");
    Serial.println(winkelRad, 4);
  }

  delay(50); // Kleines Delay für saubere Lesbarkeit im Monitor
}