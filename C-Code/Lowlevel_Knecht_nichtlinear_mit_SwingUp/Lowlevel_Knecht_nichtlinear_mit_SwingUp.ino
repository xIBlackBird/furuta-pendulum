#include <math.h>

// --- 1. LQR Parameter ---
float K[4] = {-0.0346, 0.4239, -0.0184, 0.0378};
const float max_tau = 3.0; 

// --- 2. Physik-Parameter für Swing-Up (aus MATLAB) ---
const float mp = 0.03651;
const float lp = 0.0705;
const float g = 9.81;
// J2 = theta2 + mp*lp^2 (Berechnet aus deinen MATLAB-Werten)
const float J2 = 0.000221; 
const float E_target = 2.0 * mp * g * lp; // Energie im oberen Totpunkt

// --- 3. State-Machine Schwellenwerte ---
const float CATCH_ANGLE = 0.5;     // ab ca. 17 Grad Abweichung fängt der LQR
const float CATCH_VELOCITY = 4.0;  // max. Geschwindigkeit für den LQR-Fang
String active_state = "SWING";

float x[4] = {0.0, 0.0, 0.0, 0.0};
float tau = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); 
}

void loop() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    
    int comma1 = msg.indexOf(',');
    int comma2 = msg.indexOf(',', comma1 + 1);
    int comma3 = msg.indexOf(',', comma2 + 1);
    
    if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
      x[0] = msg.substring(0, comma1).toFloat();
      x[1] = msg.substring(comma1 + 1, comma2).toFloat();
      x[2] = msg.substring(comma2 + 1, comma3).toFloat();
      x[3] = msg.substring(comma3 + 1).toFloat();
      
      // Winkel sauber auf -PI bis +PI einrollen (zur Sicherheit)
      float phi2 = x[1];
      while (phi2 > M_PI) phi2 -= 2.0 * M_PI;
      while (phi2 < -M_PI) phi2 += 2.0 * M_PI;
      
      // --- STATE MACHINE ---
      // Wenn das Pendel nah genug an 0 ist UND nicht zu schnell -> LQR
      if (fabs(phi2) < CATCH_ANGLE && fabs(x[3]) < CATCH_VELOCITY) {
          active_state = "LQR";
      } 
      // Wenn es wieder zu weit wegfällt -> Zurück zum Swing-Up
      else if (fabs(phi2) > CATCH_ANGLE + 0.1) {
          active_state = "SWING";
      }
      
      // --- REGELUNG ---
      if (active_state == "LQR") {
          // LQR-Regelgesetz (0 = Oben)
          tau = -(K[0]*x[0] + K[1]*phi2 + K[2]*x[2] + K[3]*x[3]);
      } 
      else {
          // SWING-UP REGELGESETZ
          float E_kin = 0.5 * J2 * x[3] * x[3];
          float E_pot = mp * g * lp * (1.0 + cos(phi2));
          float E_current = E_kin + E_pot;
          
          float k_swing = 5.0; // Tuning-Faktor: Wie aggressiv gepumpt wird
          
          // Richtung bestimmen: sign(Geschwindigkeit * cos(Winkel))
          float sign_term = (x[3] * cos(phi2) > 0) ? 1.0 : -1.0;
          
          tau = k_swing * (E_target - E_current) * sign_term;
      }
      
      // Motorschutz
      if (tau > max_tau)  tau = max_tau;
      if (tau < -max_tau) tau = -max_tau;
      
      // Antwort senden
      Serial.print(tau, 4);      Serial.print(";");
      Serial.print(x[0], 4);     Serial.print(";");
      Serial.print(phi2, 4);     Serial.print(";");
      Serial.print(x[2], 4);     Serial.print(";");
      Serial.print(x[3], 4);     Serial.print(";");
      Serial.println(active_state); 
    }
  }
}