import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import csv
import sys

# --- Konfiguration ---
COM_PORT = 'COM8'  # Wieder auf deinen Port anpassen!
BAUD_RATE = 500000
DAUER_SEKUNDEN = 6.0 # Dauer eines einzelnen Versuchs

print(f"Verbinde mit {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Warten auf Arduino-Neustart durch USB-Verbindung
    
    versuch_nr = 8
    
    # --- Die unendliche Versuchs-Schleife ---
    while True:
        start_input = input(f"\n=== VERSUCH {versuch_nr} ===\nDrücke ENTER zum Starten (oder tippe 'q' zum Beenden): ")
        
        if start_input.lower() == 'q':
            print("Beende Programm...")
            break
            
        # Variablen für diesen Versuch leeren
        log_t, log_state, log_x, log_u = [], [], [], []
        
        # Puffer leeren, falls noch alte Daten vom Arduino ankommen
        ser.reset_input_buffer()
        
        # Start-Signal an Arduino senden
        ser.write(b'S')
        print(f"Versuch {versuch_nr} läuft. Zeichne Daten auf...")
        
        dateiname = f'pendel_messung_versuch_{versuch_nr}.csv'
        start_time_pc = time.time()
        
        with open(dateiname, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Zeit_s', 'State', 'x1_rad', 'x2_rad', 'x3_rads', 'x4_rads', 'U_Volt'])
            
            # Daten erfassen, bis die Zeit um ist
            while (time.time() - start_time_pc) < DAUER_SEKUNDEN:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        werte = line.split(',')
                        if len(werte) == 7:
                            try:
                                t_ard = float(werte[0])
                                state = int(werte[1])
                                x_vec = [float(werte[2]), float(werte[3]), float(werte[4]), float(werte[5])]
                                u = float(werte[6])
                                
                                log_t.append(t_ard)
                                log_state.append(state)
                                log_x.append(x_vec)
                                log_u.append(u)
                                
                                writer.writerow(werte)
                            except ValueError:
                                pass # Ignoriere kaputte Zeilen (passiert manchmal beim seriellen Start)
                                
        print("Datenaufzeichnung beendet. Erstelle Plot...")
        
        # --- Auswertung und Plot ---
        if len(log_t) == 0:
            print("Keine Daten empfangen! Überprüfe die Arduino-Verbindung.")
            continue
            
        t_arr = np.array(log_t)
        t_arr = t_arr - t_arr[0] # Zeitachse bei 0 starten lassen
        x_arr = np.array(log_x)
        state_arr = np.array(log_state)
        u_arr = np.array(log_u)

        fig, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True)
        fig.canvas.manager.set_window_title(f'Auswertung - Versuch {versuch_nr}')

        axs[0].plot(t_arr, np.degrees(x_arr[:, 0]), label='Motorarm phi1')
        axs[0].set_ylabel('Winkel [Grad]')

        axs[1].plot(t_arr, np.degrees(x_arr[:, 1]), label='Pendel phi2', color='orange')
        axs[1].set_ylabel('Winkel [Grad]')

        axs[2].plot(t_arr, np.degrees(x_arr[:, 2]), label='Arm-Geschw. phi1_dot', color='blue')
        axs[2].set_ylabel('Geschw. [Grad/s]')

        axs[3].plot(t_arr, np.degrees(x_arr[:, 3]), label='Pendel-Geschw. phi2_dot', color='red')
        axs[3].set_ylabel('Geschw. [Grad/s]')

        axs[4].plot(t_arr, u_arr, label='Spannung U', color='green')
        axs[4].set_ylabel('Spannung [V]')
        axs[4].set_xlabel('Zeit [s]')

        # Umschaltpunkte einzeichnen
        transition_indices = np.where(np.diff(state_arr) != 0)[0] + 1
        added_labels = set()

        for idx in transition_indices:
            t_trans = t_arr[idx]
            new_state = state_arr[idx]
            
            if new_state == 1:
                color, label = 'gray', 'Warten (frei)'
            elif new_state == 2:
                color, label = 'red', 'Kick (-6V)'
            elif new_state == 3:
                color, label = 'blue', 'Arm zurück (+8V)'
            elif new_state == 4:
                color, label = 'magenta', 'LQR aktiv'
            else:
                continue
                
            label_for_plot = "_nolegend_" if label in added_labels else label
            added_labels.add(label)
                
            for ax in axs:
                ax.axvline(x=t_trans, color=color, linestyle='--', alpha=0.7, label=label_for_plot)

        for ax in axs:
            ax.legend(loc='upper right', fontsize='x-small')
            ax.grid(True)

        plt.tight_layout()
        
        print("-> Schließe das Plot-Fenster, um den nächsten Versuch zu starten!")
        plt.show() # Skript pausiert hier, bis das Fenster geschlossen wird
        
        versuch_nr += 1 # Zähler hochsetzen für den nächsten Versuch

except serial.SerialException as e:
    print(f"Schnittstellen-Fehler: {e}\n(Ist der Arduino eingesteckt und der Serielle Monitor geschlossen?)")
except KeyboardInterrupt:
    print("\nProgramm manuell beendet.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial Port geschlossen.")