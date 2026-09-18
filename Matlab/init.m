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
r             = 0.08;                % [m] Abstand Pendel zu Motorachse
rm            = 0.008;               % [m] Radius Motorarm
ls            = 0.128;               % [m] Länge des Pendelstabs
rs            = 0.006;               % [m] Radius Pendelstab
Vs            = pi * rs^2 * ls;      % [m^3] Volumen des Pendelstabs
dichte_pla    = 1.24 * 10^3;         % [kg/m^3] Dichte PLA
dichte_alu    = 2.7 * 10^3;          % [kg/m^3] Dichte Aluminium

mg            = 3 * 5.3 * 10^-3;     % [kg] Masse des Pendelgewichts (3x 5,3g)
lg            = ls - 0.012;          % [m] Schwerpunkt des Gewichts (1,2cm vor Ende)
ms            = Vs * dichte_alu;     % [kg] Masse des Alu-Pendelstabs
mp            = ms + mg;             % [kg] Gesamtmasse des Pendels (Stab + Gewicht)
lp            = (ms*ls/2 + mg*lg) / (ms+mg); % [m] Neuer gemeinsamer Massenschwerpunkt

% ==========================================
% 4. Trägheitsmomente
% ==========================================
theta_motor   = 0.0000036;           % [kg*m^2] Trägheitsmoment des Motors
theta_getriebe= 0;                   % [kg*m^2] Trägheitsmoment des Getriebes
theta_arm     = 176672.25 * 10^-9;   % [kg*m^2] Aus dem CAD-Modell
theta1        = theta_motor*i^2 + theta_getriebe + theta_arm; % [kg*m^2] Gesamt Arm

% Satz von Steiner (Parallelachsen-Theorem) für das Pendel:
theta_stab    = 1/12*ms*ls^2 + ms*(lp-ls/2)^2; % [kg*m^2] Stab bezogen auf neuen Schwerpunkt
theta_gewicht = 0 + mg*(lg-lp)^2;              % [kg*m^2] Gewicht bezogen auf neuen Schwerpunkt
theta2        = theta_stab + theta_gewicht;    % [kg*m^2] Trägheitsmoment um neuen Schwerpunkt

theta_1A      = theta1 + mp*r^2;     % [kg*m^2] Ersatzträgheit Motorachse
theta_2P      = theta2 + mp*lp^2;    % [kg*m^2] Ersatzträgheit Pendelachse

% ==========================================
% 5. Dynamik-Hilfsvariablen & Reibung
% ==========================================
Km = mp*r*lp;
Gp = mp*lp*g;
Det = theta_1A*theta_2P - Km^2;     % Systemdeterminante

b1 = 0;                             % Mechanische Reibung am Motor
b2 = 1.21 * theta_2P;                % Reibung Pendellagerung (gemessen!)
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

% disp('Kontinuierliche Systemmatrix Ac:'); disp(Ac);
% disp('Kontinuierliche Eingangsmatrix Bc:'); disp(Bc);

% ==========================================
% 7. Diskretisierung (zoh)
% ==========================================
sys_c = ss(Ac, Bc, Cc, Dc); 
Ts = 0.005;                         % Abtastzeit (200 Hz)
sys_d = c2d(sys_c, Ts, 'zoh');      % System diskretisieren
Ad = sys_d.A;
Bd = sys_d.B;

% disp('Diskrete Systemmatrix Ad:'); disp(Ad);
% disp('Diskrete Eingangsmatrix Bd:'); disp(Bd);

% ==========================================
% 8. LQR Regler
% ==========================================
Q = diag([0.5, 15, 0.001, 0.001]);             % Gewichtung der Zustände
R_LQR = 10;                        % Gewichtung der Stellgröße

K = dlqr(Ad, Bd, Q, R_LQR);
disp('Regler-Matrix K:'); disp(K);


% % ==========================================
% % LQR NUR FÜR DEN MOTORARM (2x2 System)
% % ==========================================
% 
% % Neues reduziertes Zustandsraummodell für [x1; x3]
% % Zeile 1: dx1/dt = x3
% % Zeile 2: dx3/dt = ( c_motor * U - b1_ges * x3 ) / theta_1A
% A_arm = [0, 1; 
%          0, -b1_ges / theta_1A];
% 
% B_arm = [0; 
%          c_motor / theta_1A];
% 
% % Diskretisierung
% sys_arm_c = ss(A_arm, B_arm, eye(2), 0);
% Ts = 0.005; % Deine 200 Hz
% sys_arm_d = c2d(sys_arm_c, Ts, 'zoh');
% Ad_arm = sys_arm_d.A;
% Bd_arm = sys_arm_d.B;
% 
% disp('Diskrete Systemmatrix Ad (nur Arm):'); disp(Ad_arm);
% disp('Diskrete Eingangsmatrix Bd (nur Arm):'); disp(Bd_arm);
% 
% % --- LQR Tuning für den Arm ---
% % Q-Matrix: [Gewicht für Position x1, Gewicht für Geschwindigkeit x3]
% Q_arm = diag([10, 1]); 
% 
% % R-Matrix: Bestraft hohen Spannungseinsatz
% R_arm = 100; 
% 
% % K-Matrix berechnen (Wird jetzt nur 2 Werte haben!)
% K_arm = dlqr(Ad_arm, Bd_arm, Q_arm, R_arm);
% disp('Regler-Matrix K_arm [K1, K3]:'); disp(K_arm);