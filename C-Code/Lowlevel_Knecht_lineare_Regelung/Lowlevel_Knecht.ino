#include <math.h>

// Deine perfekten K-Werte (für 0 = oben)
float K[4] = {-0.0346, 0.4239, -0.0184, 0.0378};
const float max_tau = 3.0; 

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
      
      // --- LQR REGELUNG ---
      // Da 0 = oben ist und x[1] dein Winkel ist, ist x[1] automatisch die Abweichung.
      // Kein Pi abziehen, keine Vorzeichen drehen. Einfach Mathematik pur:
      tau = -(K[0]*x[0] + K[1]*x[1] + K[2]*x[2] + K[3]*x[3]);
      
      // Motorschutz
      if (tau > max_tau)  tau = max_tau;
      if (tau < -max_tau) tau = -max_tau;
      
      // Antwort senden (Format: tau;x0;x1;x2;x3;LQR)
      Serial.print(tau, 4);      Serial.print(";");
      Serial.print(x[0], 4);     Serial.print(";");
      Serial.print(x[1], 4);     Serial.print(";");
      Serial.print(x[2], 4);     Serial.print(";");
      Serial.print(x[3], 4);     Serial.print(";");
      Serial.println("LQR"); 
    }
  }
}