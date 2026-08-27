import serial
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# --- 1. Deine linearen Systemmatrizen (aus MATLAB) ---
Ad = np.array([[1.0000, 0.0037, 0.0050, 0.0000], 
               [0.0000, 1.0049, 0.0000, 0.0050], 
               [0.0000, 1.4906, 1.0000, 0.0037], 
               [0.0000, 1.9560, 0.0000, 1.0049]])

Bd = np.array([0.1585, 0.1474, 63.4505, 58.9966])

# STARTZUSTAND: 0.1 rad Auslenkung von oben (0 = oben)
x = np.array([0.0, 0.1, 0.0, 0.0]) 
history_x = []

# --- NEU: Zähler für die Störung ---
step_counter = 0

# --- 2. Echtzeit-Schleife ---
if sys.platform.startswith('win'):
    port = 'COM8'
else:
    port = '/dev/ttyACM0'

print("Verbinde mit Arduino...")
try:
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2) 
    print("Starte LQR-Test mit Störung! Drücke Strg+C zum Plotten.")

    while True:
        # A) Zustand an Arduino senden
        msg_out = f"{x[0]:.4f},{x[1]:.4f},{x[2]:.4f},{x[3]:.4f}\n"
        ser.write(msg_out.encode('utf-8'))
        
        # B) Antwort vom Arduino lesen
        msg_in = ser.readline().decode('utf-8').strip()
        
        if msg_in:
            try:
                parts = msg_in.split(';')
                tau = float(parts[0])
                
                if len(parts) == 6:
                    print(f"Tau: {tau:7.4f} Nm | x = [{float(parts[1]):7.4f}, {float(parts[2]):7.4f}, {float(parts[3]):7.4f}, {float(parts[4]):7.4f}]")
                
                # --- PHYSIK-SIMULATION (Linear) ---
                x = np.dot(Ad, x) + Bd * tau
                
                # --- NEU: DIE STÖRUNG ---
                step_counter += 1
                if step_counter == 100:  # Nach ca. 1-2 Sekunden
                    print("\n=======================================================")
                    print(" BAM! 💥 STÖRUNG! Pendel bekommt einen harten Schubs!")
                    print("=======================================================\n")
                    x[3] += 5.0  # Plötzliche Winkelgeschwindigkeit von 5 rad/s addieren
                
                if step_counter == 200:  # Noch ein Schubs in die andere Richtung
                    print("\n=======================================================")
                    print(" BAM! 💥 2. STÖRUNG! Schubs in die Gegenrichtung!")
                    print("=======================================================\n")
                    x[3] -= 5.0
                
                history_x.append(x.copy())
            
            except Exception as e:
                pass 
                
except KeyboardInterrupt:
    print("\nBeendet. Generiere Animation...")
    ser.close()

# --- 3. 3D-Animation ---
if len(history_x) > 0:
    history_x = np.array(history_x)
    q1_data = history_x[:, 0] 
    q2_data = history_x[:, 1] 

    L1, L2 = 0.2, 0.3  
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-0.4, 0.4]); ax.set_ylim([-0.4, 0.4]); ax.set_zlim([-0.4, 0.4])
    ax.set_title('LQR Balancing Test (Linear, 0 = Oben) + Störungen')
    line, = ax.plot([], [], [], 'k-o', linewidth=3, markersize=8)

    def update(frame):
        q1, q2 = q1_data[frame], q2_data[frame]
        x1 = L1 * np.cos(q1)
        y1 = L1 * np.sin(q1)
        
        x2 = x1 - L2 * np.sin(q2) * np.sin(q1)
        y2 = y1 + L2 * np.sin(q2) * np.cos(q1)
        z2 = L2 * np.cos(q2) 
        
        line.set_data([0, x1, x2], [0, y1, y2])
        line.set_3d_properties([0, 0, z2])
        return line,

    ani = animation.FuncAnimation(fig, update, frames=len(q1_data), interval=5, blit=False)
    plt.show()