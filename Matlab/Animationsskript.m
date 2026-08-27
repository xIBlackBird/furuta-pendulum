%% 3D-Animation des Furuta-Pendels (Endlos-Loop)

% 1. Daten aus Simulink auslesen
t = tout;       
x = x_out;      

% Winkel extrahieren (Hier ist die + pi Korrektur für den Arbeitspunkt!)
phi1 = x(:,1); 
phi2 = x(:,2) + pi; 

% 2. 3D-Fenster vorbereiten
fig = figure('Name', 'Furuta Pendulum 3D', 'Color', 'w'); % fig abspeichern für den Loop-Check
h_arm = plot3([0,0], [0,0], [0,0], 'k-', 'LineWidth', 4); hold on;
h_pend = plot3([0,0], [0,0], [0,0], 'k-', 'LineWidth', 3);
h_mass = plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

axis([-0.2 0.2 -0.2 0.2 -0.2 0.2]);
grid on; 
view(3); 
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Simulation: Furuta-Pendel Balancieren');

% 3. Die Endlos-Schleife
while ishandle(fig) % Wiederhole, solange das Fenster existiert
    
    for i = 1:5:length(t) 
        % Falls du das Fenster mitten in der Animation schließt, brechen wir ab:
        if ~ishandle(fig) 
            break; 
        end
        
        % Kinematik
        x_j = r * cos(phi1(i));
        y_j = r * sin(phi1(i));
        z_j = 0;
        
        x_p = x_j - lp * sin(phi1(i)) * sin(phi2(i));
        y_p = y_j + lp * cos(phi1(i)) * sin(phi2(i));
        z_p = -lp * cos(phi2(i));
        
        % Plot updaten
        set(h_arm, 'XData', [0, x_j], 'YData', [0, y_j], 'ZData', [0, z_j]);
        set(h_pend, 'XData', [x_j, x_p], 'YData', [y_j, y_p], 'ZData', [z_j, z_p]);
        set(h_mass, 'XData', x_p, 'YData', y_p, 'ZData', z_p);
        
        drawnow;
        pause(0.01);
    end
    
    % Kurze Pause von einer halben Sekunde am Ende, bevor es von vorne losgeht
    if ishandle(fig)
        pause(0.5);
    end
end


% Einmal Animation
% %% 3D-Animation des Furuta-Pendels
% % Dieses Skript direkt nach der Simulink-Simulation ausführen!
% 
% % 1. Daten aus Simulink auslesen
% t = out.tout;       % Zeitvektor aus Simulink
% x = out.x_out;      % Zustands-Array aus Simulink
% 
% % Winkel extrahieren (Spalte 1 = Motorarm, Spalte 2 = Pendel)
% phi1 = x(:,1); 
% phi2 = x(:,2);
% 
% % 2. 3D-Fenster vorbereiten
% figure('Name', 'Furuta Pendulum 3D', 'Color', 'w');
% % Alles in Schwarz gehalten
% h_arm = plot3([0,0], [0,0], [0,0], 'k-', 'LineWidth', 4); hold on;
% h_pend = plot3([0,0], [0,0], [0,0], 'k-', 'LineWidth', 3);
% h_mass = plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
% 
% % Achsen und Ansicht formatieren (Werte an deine Längen angepasst)
% axis([-0.2 0.2 -0.2 0.2 -0.2 0.2]);
% grid on; 
% view(3); % 3D Ansicht
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Simulation: Furuta-Pendel Balancieren');
% 
% % 3. Animations-Schleife
% % Wir überspringen ein paar Datenpunkte (Schrittweite 5), damit die 
% % Animation in Echtzeit läuft und nicht in Zeitlupe.
% for i = 1:5:length(t) 
% 
%     % Kinematik: Gelenkposition (Ende des Motorarms)
%     x_j = r * cos(phi1(i));
%     y_j = r * sin(phi1(i));
%     z_j = 0;
% 
%     % Kinematik: Pendelschwerpunkt (Aus unserer Herleitung!)
%     x_p = x_j - lp * sin(phi1(i)) * sin(phi2(i));
%     y_p = y_j + lp * cos(phi1(i)) * sin(phi2(i));
%     z_p = -lp * cos(phi2(i));
% 
%     % Linien im Plot updaten
%     set(h_arm, 'XData', [0, x_j], 'YData', [0, y_j], 'ZData', [0, z_j]);
%     set(h_pend, 'XData', [x_j, x_p], 'YData', [y_j, y_p], 'ZData', [z_j, z_p]);
%     set(h_mass, 'XData', x_p, 'YData', y_p, 'ZData', z_p);
% 
%     drawnow;
%     pause(0.01);
% end