# Furuta-Pendel

Ein Forschungs- und Entwicklungsprojekt zum Aufbau und zur Regelung eines Furuta-Pendels. Das Projekt kombiniert mechanische Konstruktion, Messdatenerfassung, Simulation und Regelungsalgorithmen mit Arduino-ähnlicher Mikrocontroller-Programmierung sowie MATLAB/Python-Analyse.

## Überblick

Das Ziel dieses Projekts ist die Umsetzung und Untersuchung eines Furuta-Pendels als Regelungsaufgabe mit Fokus auf:

- Stabilisierung der Aufrechte (Balancing)
- Swing-Up-Verhalten
- Messung und Auswertung von Zuständen
- Regelungsentwurf mit linearer und nichtlinearer Dynamik
- Hardware-in-the-Loop (HIL)- und Software-in-the-Loop (SIL)-Tests
- mechanische Konstruktion und CAD-Modelle

## Projektstruktur

```text
furuta-pendulum/
├── C-Code/
│   ├── AS5600_Test/
│   ├── LQR/
│   ├── LQR_Sequenz_Logging/
│   ├── LQR_Sequenz_Logging_Button/
│   ├── Lowlevel_Knecht_lineare_Regelung/
│   ├── Lowlevel_Knecht_nichtlinear_mit_SwingUp/
│   ├── Motor_Test/
│   ├── Motorencoder/
│   ├── max_Drehmoment_bestimmen/
│   └── measure_states/
├── CAD/
│   ├── STL- und SLDPRT-Dateien für Mechanik und Bauteile
│   └── Zusammengesetzter Motorarm und weitere CAD-Komponenten
├── Matlab/
│   ├── init.m
│   ├── Animationsskript.m
│   ├── Simulation.slx
│   └── weitere Simulations- und Cache-Dateien
├── Python/
│   ├── HIL energy_test.py
│   ├── HIL lineare System.py
│   ├── HIL max Drehmoment.py
│   ├── HIL nichtlinear SwingUp.py
│   ├── HIL nichtlineares System.py
│   ├── SIL swing_up test.py
│   ├── read_csv.py
│   └── start_pendulum.py
├── .gitignore
├── .gitattributes
└── README.md
