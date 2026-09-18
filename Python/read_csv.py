import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. Konfiguration ---
DATEINAME = 'pendel_messung_versuch_3.csv'

# --- 2. Daten laden ---
try:
    df = pd.read_csv(DATEINAME)
except FileNotFoundError:
    print(f"Fehler: Die Datei '{DATEINAME}' wurde nicht im aktuellen Ordner gefunden.")
    exit()

# Zeitachse bei 0 starten lassen
t = df['Zeit_s'] - df['Zeit_s'].iloc[0]
state_arr = df['State'].values

# --- 3. Plot erstellen ---
fig, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True)
fig.canvas.manager.set_window_title(f'Offline Auswertung - {DATEINAME}')

# Motorarm
axs[0].plot(t, np.degrees(df['x1_rad']), label='Motorarm phi1')
axs[0].set_ylabel('Winkel [Grad]')

# Pendel
axs[1].plot(t, np.degrees(df['x2_rad']), label='Pendel phi2', color='orange')
axs[1].set_ylabel('Winkel [Grad]')

# Arm-Geschwindigkeit
axs[2].plot(t, np.degrees(df['x3_rads']), label='Arm-Geschw. phi1_dot', color='blue')
axs[2].set_ylabel('Geschw. [Grad/s]')

# Pendel-Geschwindigkeit
axs[3].plot(t, np.degrees(df['x4_rads']), label='Pendel-Geschw. phi2_dot', color='red')
axs[3].set_ylabel('Geschw. [Grad/s]')

# Spannung
axs[4].plot(t, df['U_Volt'], label='Spannung U', color='green')
axs[4].set_ylabel('Spannung [V]')
axs[4].set_xlabel('Zeit [s]')

# --- 4. Umschaltpunkte (States) einzeichnen ---
transition_indices = np.where(np.diff(state_arr) != 0)[0] + 1
added_labels = set()

for idx in transition_indices:
    t_trans = t.iloc[idx]
    new_state = state_arr[idx]
    
    if new_state == 1:
        color, label = 'gray', 'Warten (frei)'
    elif new_state == 2:
        color, label = 'red', 'Kick (-6V)'
    elif new_state == 3:
        color, label = 'blue', 'Arm zurück (+8V)'
    elif new_state == 4:
        color, label = 'magenta', 'LQR aktiv'
    elif new_state == 5:
        color, label = 'black', 'NOT-AUS'
    else:
        continue
        
    label_for_plot = "_nolegend_" if label in added_labels else label
    added_labels.add(label)
        
    for ax in axs:
        ax.axvline(x=t_trans, color=color, linestyle='--', alpha=0.7, label=label_for_plot)

# --- 5. Layout anwenden ---
for ax in axs:
    ax.legend(loc='upper right', fontsize='x-small')
    ax.grid(True)

plt.tight_layout()
plt.show()