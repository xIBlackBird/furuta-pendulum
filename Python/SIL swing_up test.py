# %% ZELLE 1: Parameter und Setup
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# --- 1. Systemparameter (Hier deine echten Werte aus MATLAB eintragen!) ---
Theta_1A = 0.0038
Theta_2P = 0.00042
m_p = 0.055
r = 0.08
l_p = 0.079
g = 9.81
c_motor = 0.1144
b1 = 0.0286
b2 = 0.000517

# LQR K-Matrix (aus MATLAB kopieren)
K = np.array([-0.2126, 15.4537, -0.5591, 1.4222]) 

# %% ZELLE 2: Dynamik und Regler
# --- 2. Nicht-lineare Bewegungsgleichungen (Die Strecke) ---
def pendel_dynamik(t, x, U):
    phi1, phi2, phi1_dot, phi2_dot = x
    
    # Massenmatrix M(q)
    M11 = Theta_1A + Theta_2P * np.sin(phi2)**2
    M12 = -m_p * r * l_p * np.cos(phi2)
    M21 = -m_p * r * l_p * np.cos(phi2)
    M22 = Theta_2P
    M = np.array([[M11, M12], [M21, M22]])
    
    # Vektor der Coriolis-, Zentrifugal- und Gravitationskräfte C(q, q_dot)
    C1 = 2 * Theta_2P * np.sin(phi2) * np.cos(phi2) * phi1_dot * phi2_dot + m_p * r * l_p * np.sin(phi2) * phi2_dot**2
    C2 = -Theta_2P * np.sin(phi2) * np.cos(phi2) * phi1_dot**2 - m_p * g * l_p * np.sin(phi2)
    C = np.array([C1, C2])
    
    # Generalisierte Kräfte Q
    tau1 = c_motor * U - b1 * phi1_dot
    tau2 = -b2 * phi2_dot
    Q = np.array([tau1, tau2])
    
    # Gleichungssystem auflösen: M * q_ddot + C = Q  =>  q_ddot = M^-1 * (Q - C)
    q_ddot = np.linalg.solve(M, Q - C)
    
    return [phi1_dot, phi2_dot, q_ddot[0], q_ddot[1]]

# Globale Variablen für den Zustandsautomaten
controller_state = 0 
last_phi2_dot = 0.0

def controller(t, x):
    global controller_state, last_phi2_dot
    
    phi1, phi2, phi1_dot, phi2_dot = x
    phi2_norm = (phi2 + np.pi) % (2 * np.pi) - np.pi
    
    # 1. Bedingung prüfen: LQR-Fangbereich erreicht?
    if abs(phi2_norm) < 0.35:
        controller_state = 4 # WICHTIG: Auf 4 geändert! (LQR-Phase)
        
    U = 0.0
    
    # 2. State-Machine für Swing-Up und Regelung
    if controller_state == 0:
        # PHASE 0: Initialer Schubser (für 200 ms)
        U = 4.0
        if t > 0.1:
            controller_state = 1
            
    elif controller_state == 1:
        # PHASE 1: Motor ausrollen lassen, auf Peak (Umkehrpunkt) warten
        U = 0.0
        # Nulldurchgang der Geschwindigkeit prüfen
        if phi2_dot * last_phi2_dot <= 0.0:
            controller_state = 2
            
    elif controller_state == 2:
        # PHASE 2: Arm aggressiv auf -40 Grad ziehen, aber schon vorher bremsen, wenn -10 Grad erreicht sind
        if phi1 > np.radians(-10): 
            U = -6.0
        else:
            U = 0.0 # Arm bremsen, wenn -40 erreicht ist

        if phi1 < np.radians(-60): 
            controller_state = 3
            
    elif controller_state == 3:
        # PHASE 3: Arm zurück auf 0 Grad ziehen
        if phi1 < np.radians(-25): 
            U = 8.0
        else:
            U = 0.0 # Arm bremsen, wenn 0 erreicht ist
            
        # NEU: Wenn das Pendel seinen Schwung verliert und wieder umdreht, 
        # fangen wir einfach von vorne an zu pumpen!
        # if phi2_dot * last_phi2_dot <= 0.0:
        #     controller_state = 1

    elif controller_state == 4:
        # PHASE 4: LQR aktiv
        x_regler = np.array([phi1, phi2_norm, phi1_dot, phi2_dot])
        U = -np.dot(K, x_regler)
        
    # Geschwindigkeit für den nächsten Zyklus merken
    last_phi2_dot = phi2_dot
    
    return np.clip(U, -12.0, 12.0)

# %% ZELLE 3: Simulation und Plot
Ts = 0.005  
t_end = 5.0 
steps = int(t_end / Ts)

# Pendel startet hängend
x_aktuell = np.array([0.0, np.pi, 0.0, 0.0]) 

# Variablen zurücksetzen, falls die Zelle mehrfach ausgeführt wird
controller_state = 0
last_phi2_dot = 0.0

log_t = []
log_x = []
log_u = []
log_state = [] # NEU: Speichert die aktuelle Phase der State-Machine

for i in range(steps):
    t_start = i * Ts
    t_stop = t_start + Ts
    
    # 1. Regler berechnet Spannung
    U = controller(t_start, x_aktuell)
    
    # 2. Strecke simuliert
    sol = solve_ivp(pendel_dynamik, [t_start, t_stop], x_aktuell, args=(U,), method='RK45')
    
    # 3. Nächster Zustand
    x_aktuell = sol.y[:, -1]
    
    # 4. Daten loggen
    log_t.append(t_start)
    log_x.append(x_aktuell)
    log_u.append(U)
    log_state.append(controller_state) # NEU: Zustand mitschreiben

# --- 5. Auswertung / Plot ---
log_x = np.array(log_x)
log_state = np.array(log_state)

# Auf 5 Subplots erhöhen und die Figure etwas höher machen (15 statt 12)
fig, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True)

# 1. Daten plotten: Motorarm Winkel
axs[0].plot(log_t, np.degrees(log_x[:, 0]), label='Motorarm phi1')
axs[0].set_ylabel('Winkel [Grad]')

# 2. Daten plotten: Pendel Winkel
axs[1].plot(log_t, np.degrees(log_x[:, 1]), label='Pendel phi2', color='orange')
axs[1].set_ylabel('Winkel [Grad]')

# 3. Daten plotten: Motorarm Geschwindigkeit
axs[2].plot(log_t, np.degrees(log_x[:, 2]), label='Arm-Geschw. phi1_dot', color='blue')
axs[2].set_ylabel('Geschw. [Grad/s]')

# 4. Daten plotten: Pendel Geschwindigkeit
axs[3].plot(log_t, np.degrees(log_x[:, 3]), label='Pendel-Geschw. phi2_dot', color='red')
axs[3].set_ylabel('Geschw. [Grad/s]')

# 5. Daten plotten: Spannung U
axs[4].plot(log_t, log_u, label='Spannung U', color='green')
axs[4].set_ylabel('Spannung [V]')
axs[4].set_xlabel('Zeit [s]')

# --- Markierungen für Umschaltpunkte einzeichnen ---
transition_indices = np.where(np.diff(log_state) != 0)[0] + 1
added_labels = set() # Verhindert doppelte Einträge in der Legende

for idx in transition_indices:
    t_trans = log_t[idx]
    new_state = log_state[idx]
    
    # Farben und Beschriftungen je nach Umschaltpunkt festlegen
    if new_state == 1:
        color, label = 'gray', 'Warten (frei)'
    elif new_state == 2:
        color, label = 'red', 'Kick (-5V)'
    elif new_state == 3:
        color, label = 'blue', 'Arm zurück (+5V)'
    elif new_state == 4:
        color, label = 'magenta', 'LQR aktiv'
    else:
        continue
        
    # Sicherstellen, dass das Label nur einmal in der Legende landet
    label_for_plot = "_nolegend_" if label in added_labels else label
    added_labels.add(label)
        
    # Senkrechte Linie über ALLE 5 Subplots zeichnen
    for ax in axs:
        ax.axvline(x=t_trans, color=color, linestyle='--', alpha=0.7, label=label_for_plot)

# Layout und Legenden anwenden
for ax in axs:
    # Legende außerhalb oder klein halten, damit sie nicht den Graphen verdeckt
    ax.legend(loc='upper right', fontsize='x-small')
    ax.grid(True)

plt.tight_layout()
plt.show()