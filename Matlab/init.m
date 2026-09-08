% ==========================================
% 1. Grund- und Getriebeparameter
% ==========================================
g = 9.81;                           % Erdbeschleunigung
i = 30;                             % Übersetzung des Getriebes

% ==========================================
% 2. Motorkonstanten
% ==========================================
U_nom = 12.0;                       % Nennspannung (V)
I_stall = 5.5;                      % Blockierstrom (A)
tau_stall = 1.373;                  % Blockiermoment nach Getriebe (Nm)

R = U_nom / I_stall;                % Klemmenwiderstand (Ohm)
Kt = (tau_stall / i) / I_stall;     % Drehmomentkonstante VOR Getriebe (Nm/A)
Ke = Kt;                            % In SI-Einheiten ist Kt numerisch identisch zu Ke!
b_motor = (Kt * Ke * i^2) / R;      % Viskose Dämpfung durch den Motor (Back-EMF)
c_motor = (Kt * i) / R;             % Umrechnungsfaktor von Spannung zu Drehmoment am Arm

% ==========================================
% 3. Geometrie und Massen
% ==========================================
r = 0.08;                           % Abstand Pendel zu Motorachse
rm = 0.008;                         % Radius Motorarm
ls = 0.128;                         % Länge des Pendelstabs
rs = 0.006;                         % Radius Pendelstab
Vs = pi*rs^2*ls;
dichte_pla = 1.24 * 10^3;
dichte_alu = 2.7 * 10^3;
mg = 0.0;                           % Masse des Pendelgewichts
ms = Vs * dichte_alu;
mp = ms+mg;                         % Gesamtgewicht am Pendelstab
lp = (ms*ls/2 + mg*ls)/(ms+mg);     % Neuer Massenschwerpunkt am Pendelstab

% ==========================================
% 4. Trägheitsmomente
% ==========================================
theta_motor = 0.0000036;            % Trägheitsmoment des Motors
theta_getriebe = 0;                 % Trägheitsmoment des Getriebes
theta_arm = 176672.25 * 10^-9;      % Aus dem CAD-Modell

theta1 = theta_motor*i^2 + theta_getriebe + theta_arm; 
theta_stab = 1/12*ms*ls^2 + ms*(lp-ls/2)^2; 
theta_gewicht = 0 + mg*(ls-lp)^2; 
theta2 = theta_stab + theta_gewicht; 

theta_1A = theta1 + mp*r^2; 
theta_2P = theta2 + mp*lp^2;

% ==========================================
% 5. Dynamik-Hilfsvariablen & Reibung
% ==========================================
Km = mp*r*lp;
Gp = mp*lp*g;
Det = theta_1A*theta_2P - Km^2;     % Systemdeterminante

b1 = 0;                             % Mechanische Reibung am Motor
b2 = 1.3 * theta_2P;                % Reibung Pendellagerung (gemessen!)
b1_ges = b1 + b_motor;              % Gesamtdämpfung Arm

% ==========================================
% 6. Zustandsraummodell (Kontinuierlich)
% ==========================================
Ac = [0, 0,                 1,                      0; 
      0, 0,                 0,                      1;
      0, (Km*Gp)/Det,       -theta_2P*b1_ges/Det,   -Km*b2/Det;
      0, (theta_1A*Gp)/Det, -Km*b1_ges/Det,         -theta_1A*b2/Det];

Bc = [0; 
      0;
      theta_2P*c_motor/Det;
      Km*c_motor/Det];

Cc = eye(4);
Dc = 0;

disp('Kontinuierliche Systemmatrix Ac:'); disp(Ac);
disp('Kontinuierliche Eingangsmatrix Bc:'); disp(Bc);

% ==========================================
% 7. Diskretisierung (zoh)
% ==========================================
sys_c = ss(Ac, Bc, Cc, Dc); 
Ts = 0.005;                         % Abtastzeit (200 Hz)
sys_d = c2d(sys_c, Ts, 'zoh');      % System diskretisieren
Ad = sys_d.A;
Bd = sys_d.B;

disp('Diskrete Systemmatrix Ad:'); disp(Ad);
disp('Diskrete Eingangsmatrix Bd:'); disp(Bd);

% ==========================================
% 8. LQR Regler
% ==========================================
Q = diag([50, 100, 1, 1]);          % Gewichtung der Zustände
R_LQR = 1;                          % Gewichtung der Stellgröße

K = dlqr(Ad, Bd, Q, R_LQR);
disp('Regler-Matrix K:'); disp(K);