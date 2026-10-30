from pathlib import Path
import csv
import time

import matplotlib.pyplot as plt
import numpy as np
import serial


# --- Konfiguration ---
COM_PORT = "COM9"
BAUD_RATE = 500000
DAUER_SEKUNDEN = 10.0
AUSGABEORDNER = Path(__file__).resolve().parent / "messdaten"

CSV_KOPFZEILE = [
    "Zeit_s",
    "State",
    "x1_rad",
    "x2_rad",
    "x3_rads",
    "x4_rads",
    "U_Volt",
]


def naechste_versuchsnummer():
    vorhandene_nummern = []
    for datei in AUSGABEORDNER.glob("pendel_messung_versuch_*.csv"):
        try:
            vorhandene_nummern.append(int(datei.stem.rsplit("_", 1)[1]))
        except ValueError:
            continue
    return max(vorhandene_nummern, default=0) + 1


def plotte_messung(zeit, states, zustaende, spannung, versuch_nr):
    zeit = zeit - zeit[0]
    winkel = np.degrees(zustaende)
    winkel[:, 1] = np.degrees(np.unwrap(zustaende[:, 1]))

    fig, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True)
    try:
        fig.canvas.manager.set_window_title(f"Auswertung - Versuch {versuch_nr}")
    except AttributeError:
        pass

    axs[0].plot(zeit, winkel[:, 0], label="Motorarm phi1")
    axs[0].set_ylabel("Winkel [Grad]")

    axs[1].plot(zeit, winkel[:, 1], label="Pendel phi2", color="orange")
    axs[1].set_ylabel("Winkel [Grad]")

    axs[2].plot(zeit, winkel[:, 2], label="Arm-Geschw. phi1_dot", color="blue")
    axs[2].set_ylabel("Geschw. [Grad/s]")

    axs[3].plot(zeit, winkel[:, 3], label="Pendel-Geschw. phi2_dot", color="red")
    axs[3].set_ylabel("Geschw. [Grad/s]")

    axs[4].plot(zeit, spannung, label="Spannung U", color="green")
    axs[4].set_ylabel("Spannung [V]")
    axs[4].set_xlabel("Zeit [s]")

    zustandsinfos = {
        1: ("gray", "Warten (frei)"),
        2: ("red", "Kick (-6V)"),
        3: ("blue", "Arm zurueck (+8V)"),
        4: ("magenta", "LQR aktiv"),
        5: ("black", "NOT-AUS"),
    }
    wechsel = np.where(np.diff(states) != 0)[0] + 1
    verwendete_labels = set()

    for index in wechsel:
        farbe_und_label = zustandsinfos.get(states[index])
        if farbe_und_label is None:
            continue
        farbe, label = farbe_und_label
        plot_label = label if label not in verwendete_labels else "_nolegend_"
        verwendete_labels.add(label)
        for ax in axs:
            ax.axvline(
                x=zeit[index],
                color=farbe,
                linestyle="--",
                alpha=0.7,
                label=plot_label,
            )

    for ax in axs:
        ax.legend(loc="upper right", fontsize="x-small")
        ax.grid(True)

    plt.tight_layout()
    print("Schliesse das Plot-Fenster, um den naechsten Versuch zu starten.")
    plt.show()
    plt.close(fig)


def zeichne_versuch_auf(ser, versuch_nr):
    log_t = []
    log_state = []
    log_x = []
    log_u = []

    ser.reset_input_buffer()
    ser.write(b"S")
    print(f"Versuch {versuch_nr} laeuft. Zeichne Daten auf...")

    dateipfad = AUSGABEORDNER / f"pendel_messung_versuch_{versuch_nr}.csv"
    startzeit = time.monotonic()

    with dateipfad.open(mode="w", newline="", encoding="utf-8") as datei:
        writer = csv.writer(datei)
        writer.writerow(CSV_KOPFZEILE)

        while time.monotonic() - startzeit < DAUER_SEKUNDEN:
            if ser.in_waiting == 0:
                continue

            zeile = ser.readline().decode("utf-8", errors="ignore").strip()
            werte = zeile.split(",")
            if len(werte) != 7:
                continue

            try:
                zeit = float(werte[0])
                state = int(werte[1])
                zustand = [float(wert) for wert in werte[2:6]]
                spannung = float(werte[6])
            except ValueError:
                continue

            log_t.append(zeit)
            log_state.append(state)
            log_x.append(zustand)
            log_u.append(spannung)
            writer.writerow(werte)

    print(f"Daten gespeichert: {dateipfad}")
    if not log_t:
        print("Keine gueltigen Daten empfangen. Bitte die Arduino-Verbindung pruefen.")
        return

    plotte_messung(
        np.array(log_t),
        np.array(log_state),
        np.array(log_x),
        np.array(log_u),
        versuch_nr,
    )


def main():
    AUSGABEORDNER.mkdir(parents=True, exist_ok=True)
    versuch_nr = naechste_versuchsnummer()

    print(f"Verbinde mit {COM_PORT}...")
    try:
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(2)
            while True:
                eingabe = input(
                    f"\n=== VERSUCH {versuch_nr} ===\n"
                    "ENTER zum Starten, 'q' zum Beenden: "
                )
                if eingabe.lower() == "q":
                    break

                zeichne_versuch_auf(ser, versuch_nr)
                versuch_nr += 1
    except serial.SerialException as fehler:
        print(f"Schnittstellen-Fehler: {fehler}")
        print("Ist der Arduino eingesteckt und der serielle Monitor geschlossen?")
    except KeyboardInterrupt:
        print("\nProgramm manuell beendet.")


if __name__ == "__main__":
    main()
