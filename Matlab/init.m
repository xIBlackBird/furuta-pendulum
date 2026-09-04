% Parameter definieren
g = 9.81; % Erdbeschleunigung

r = 0.09; % Länge des der Welle ab Ursprung
rm = 0.008; % Radius Motorarm
ls = 0.128; % Länge des Pendelstabs
rs = 0.006; % Radius Pendelstab

Vs = pi*rs^2*ls;

dichte_pla = 1.24 * 10^3;
dichte_alu = 2.7 * 10^3;

% m1 = 0.01721; % Masse des Motorarms, Mischung aus allen Teilen, die
% verbaut sind, Schrauben, Muttern, ...
mg = 0.0; % Masse des Pendelgewichts, Mutter oder Eisenkugel

% m1 = Vm * dichte_pla;
ms = Vs * dichte_alu;

mp = ms+mg; % Gesamtgewicht am Pendelstab
lp = (ms*ls/2 + mg*ls)/(ms+mg); % Neuer Massenschwerpunkt am Pendelstab

theta_motor = 0.0000036; % Trägheitsmoment des Motors (Rotor geschätzt als massiver drehender Zylinder von Gemini mit so ein paar groben Werten)
theta_getriebe = 0; % Trägheitsmoment des Getriebes
i = 30; % Übersetzung des Getriebes
theta_arm = 176672.25 * 10^-9; % Aus dem CAD-Modell entnommen (Davor die Schätzung mit: 1/3 * m1 * r^2; % Massenträgheitsmoment Motorarm, direkte Formel für homogene Stäbe gelagert an einem Ende (1/12*m*r^2 + m*(r/2)^2))
theta1 = theta_motor*i^2 + theta_getriebe + theta_arm; % konstantes Massenträgheitsmoment um Motorachse (schwingendes Pendel geht automatisch durch Geschwindigkeit des Massenschwerpunktes bei Lagrange mit ein)

theta_stab = 1/12*ms*ls^2 + ms*(lp-ls/2)^2; % Trägheitsmoment um Schwerpunkt + Steiner zum neuen Schwerpunkt
theta_gewicht = 0 + mg*(ls-lp)^2; % Massenträgheitsmoment Gewicht (als Punktmasse angenommen) + Steiner zum neuen Schwerpunkt
theta2 = theta_stab + theta_gewicht; % Massenträgheitsmoment um Schwerpunkt des Pendels

% Konstanten für Linearisierung zum Vereinfachen
J1 = theta1 + mp*r^2; 
J2 = theta2 + mp*lp^2;
Km = mp*r*lp;
Gp = mp*lp*g;

Det = J1*J2 - Km^2; % Systemdeterminante

% Systemmatrix
Ac = [0, 0, 1, 0; 
     0, 0, 0, 1;
     0, (Km*Gp)/Det, 0, 0;
     0, (J1*Gp)/Det, 0, 0];

% Eingangmatrix
Bc = [0; 
     0;
     J2/Det;
     Km/Det;
     ];

% Ausgangsmatrix
Cc = eye(4);

% Durchgriffsmatrix
Dc = 0;

% Anfangswerte
init_conditions = [0, 0.3, 0, 0];

disp('Matrix A:'); disp(Ac);
disp('Matrix B:'); disp(Bc);

sys_c = ss(Ac, Bc, Cc, Dc); % Kontinuierliches System

Ts = 0.005; % Abtastzeit

sys_d = c2d(sys_c, Ts, 'zoh'); % System diskretisieren

Ad = sys_d.A;
Bd = sys_d.B;
disp('Diskrete Systemmatrix Ad:'); disp(Ad);
disp('Diskrete Eingangsmatrix Bd:'); disp(Bd);

% --- LQR Regler berechnen ---
% Q-Matrix (Gewichtung der Zustände: phi1, phi2, phi_p1, phi_p2)
% Wir bestrafen Abweichungen des Pendelwinkels (Index 2) am stärksten:
Q = diag([50, 100, 1, 1]); 

% R-Matrix (Gewichtung der Stellgröße)
% Bestraft zu hohen Drehmoment-Einsatz des Motors:
R = 1; 

% Regler-Matrix K berechnen
K = dlqr(Ad, Bd, Q, R);
disp('Regler-Matrix K:'); disp(K);