#include <math.h>

// --- 1. Neuer LQR Regler (Aluminium) ---
float K[4] = {-0.1254, 1.4415, -0.0657, 0.1227};
const float max_tau = 3.0; // Sättigungsgrenze
float k_swing = 5.0; // Aggressivität

// --- 2. Physik-Parameter für Alu-Swing-Up ---
const float g = 9.81;
const float r = 0.08;
const float rm = 0.01;
const float ls = 0.1;
const float rs = 0.01;
const float mg = 0.015;
const float dichte_alu = 2700.0;

// Berechnungen exakt wie in MATLAB
const float Vm = M_PI * rm * rm * r;
const float Vs = M_PI * rs * rs * ls;
const float ms = Vs * dichte_alu;
const float mp = ms + mg;
const float lp = (ms * ls / 2.0 + mg * ls) / mp;

const float theta_stab = (1.0/12.0) * ms * ls * ls + ms * pow(lp - ls/2.0, 2);
const float theta_gewicht = mg * pow(ls - lp, 2);
const float theta2 = theta_stab + theta_gewicht;
const float J2 = theta2 + mp * lp * lp;

// Das energetische Ziel für den oberen Totpunkt
const float E_target = 2.0 * mp * g * lp;

// --- 3. State-Machine Schwellenwerte ---
const float CATCH_ANGLE = 0.5;    // Größeres Fangfenster für Alu
const float CATCH_VELOCITY = 4.0;  
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
      
      float phi2 = x[1];
      while (phi2 > M_PI) phi2 -= 2.0 * M_PI;
      while (phi2 < -M_PI) phi2 += 2.0 * M_PI;
      
      // State Machine
      if (fabs(phi2) < CATCH_ANGLE && fabs(x[3]) < CATCH_VELOCITY) {
          active_state = "LQR";
      } 
      else if (fabs(phi2) > CATCH_ANGLE + 0.1) {
          active_state = "SWING";
      }
      
      // Regelung
      if (active_state == "LQR") {
          tau = -(K[0]*x[0] + K[1]*phi2 + K[2]*x[2] + K[3]*x[3]);
      } 
      else {
          float E_kin = 0.5 * J2 * x[3] * x[3];
          float E_pot = mp * g * lp * (1.0 + cos(phi2));
          float E_current = E_kin + E_pot;
          
          float sign_term = (x[3] * cos(phi2) > 0) ? 1.0 : -1.0;
          
          tau = k_swing * (E_target - E_current) * sign_term;
      }
      
      if (tau > max_tau)  tau = max_tau;
      if (tau < -max_tau) tau = -max_tau;
      
      Serial.print(tau, 4);      Serial.print(";");
      Serial.print(x[0], 4);     Serial.print(";");
      Serial.print(phi2, 4);     Serial.print(";");
      Serial.print(x[2], 4);     Serial.print(";");
      Serial.print(x[3], 4);     Serial.print(";");
      Serial.println(active_state); 
    }
  }
}