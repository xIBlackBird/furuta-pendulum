import serial
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# STARTZUSTAND: Unten (Pi) mit minimalem Schubs
x = np.array([0.0, np.pi + 0.01, 0.0, 0.0]) 
history_x = []

def system_dynamics(x_state, tau_val):
    phi1, phi2, dphi1, dphi2 = x_state
    
    g = 9.81
    m1 = 0.01721
    ms = 0.02151
    mg = 0.015
    r = 0.08
    ls = 0.1
    
    mp = ms + mg
    lp = (ms * ls / 2.0 + mg * ls) / mp
    
    theta_motor = 0.00001
    theta_arm = (1.0/3.0) * m1 * r**2
    theta1 = theta_motor + theta_arm
    
    theta_stab = (1.0/12.0) * ms * ls**2 + ms * (lp - ls/2.0)**2
    theta_gewicht = 0.0 + mg * (ls - lp)**2
    theta2 = theta_stab + theta_gewicht
    
    M11 = theta1 + mp * r**2 + mp * lp**2 * np.sin(phi2)**2
    M12 = -mp * r * lp * np.cos(phi2)
    M21 = -mp * r * lp * np.cos(phi2)
    M22 = theta2 + mp * lp**2
    
    M = np.array([[M11, M12], 
                  [M21, M22]])
    
    # Hier wieder minimale Dämpfung rein, damit der Swing-Up realistisch ist
    b1 = 0.005 # Motordämpfung
    b2 = 0.0001 # Gelenkreibung
    
    F1 = tau_val - mp * lp**2 * np.sin(2*phi2) * dphi1 * dphi2 + mp * r * lp * np.sin(phi2) * dphi2**2 - b1 * dphi1
    F2 = 0.5 * mp * lp**2 * np.sin(2*phi2) * dphi1**2 + mp * g * lp * np.sin(phi2) - b2 * dphi2
    
    ddq = np.linalg.solve(M, np.array([F1, F2]))
    return np.array([dphi1, dphi2, ddq[0], ddq[1]])

def rk4_step(x_state, tau_val, dt=0.005):
    k1 = system_dynamics(x_state, tau_val)
    k2 = system_dynamics(x_state + 0.5 * dt * k1, tau_val)
    k3 = system_dynamics(x_state + 0.5 * dt * k2, tau_val)
    k4 = system_dynamics(x_state + dt * k3, tau_val)
    return x_state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

if sys.platform.startswith('win'):
    port = 'COM8'
else:
    port = '/dev/ttyACM0'

print("Verbinde mit Arduino...")
try:
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2) 
    print("Starte Komplette HiL-Simulation (Swing-Up -> LQR)! Drücke Strg+C zum Plotten.")

    while True:
        # Winkel auf [-pi, pi] einrollen BEVOR gesendet wird, damit Arduino glatte Werte hat
        x[1] = (x[1] + np.pi) % (2 * np.pi) - np.pi
        x[0] = (x[0] + np.pi) % (2 * np.pi) - np.pi
        
        msg_out = f"{x[0]:.4f},{x[1]:.4f},{x[2]:.4f},{x[3]:.4f}\n"
        ser.write(msg_out.encode('utf-8'))
        
        msg_in = ser.readline().decode('utf-8').strip()
        
        if msg_in:
            try:
                parts = msg_in.split(';')
                tau = float(parts[0])
                mode = parts[5] if len(parts) > 5 else "???"
                
                if len(parts) == 6:
                    print(f"Modus: {mode:5s} | Tau: {tau:7.4f} Nm | x = [{float(parts[1]):7.4f}, {float(parts[2]):7.4f}, {float(parts[3]):7.4f}, {float(parts[4]):7.4f}]")
                
                x = rk4_step(x, tau, dt=0.005)
                history_x.append(x.copy())
            
            except Exception as e:
                pass 
                
except KeyboardInterrupt:
    print("\nBeendet. Generiere Animation...")
    ser.close()

# --- 3D-Animation ---
if len(history_x) > 0:
    history_x = np.array(history_x)
    q1_data = history_x[:, 0] 
    q2_data = history_x[:, 1] 

    L1, L2 = 0.2, 0.3  
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-0.4, 0.4]); ax.set_ylim([-0.4, 0.4]); ax.set_zlim([-0.4, 0.4])
    ax.set_title('Swing-Up & Balancing')
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