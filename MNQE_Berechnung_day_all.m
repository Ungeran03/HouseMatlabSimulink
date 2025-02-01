% MATLAB-Skript: Signalfilterung mit Kalman-Filter und MNQE-Berechnung

% 1. Initialisierung
clc;
clear;

% Zeitparameter
ts = 0.1;  % Abtastzeit
T = 24;    % Gesamtdauer eines Tages [Stunden]
days = 50; % Anzahl der Tage
t = 0:ts:T; % Zeitvektor für einen Tag
analyzedDay = 1; % Tag der analysiert wird
failureLimit = 0.2; % Fehler-Schwellwert 

% Generierung des Referenzsignals (ideale Solltemperatur)
ref_signal = 22 + 2 * sin(2 * pi * t / 24); % Sinusförmige Solltemperatur über einen Tag

% Generierung der geregelten Temperatur (ohne Noise)
controlled_signal = ref_signal + 0.00002 * cos(2 * pi * t / 12) + 0.2 * sin(2 * pi * t / 6); % Geregelte Temperatur mit struktureller Verzerrung



% Generierung des verrauschten Messsignals
noise = 0.5 * randn(size(t));
measured_signal = controlled_signal + noise; % Verrauschtes Signal

% 2. Kalman-Filter Implementierung
% Initialisierung der Variablen
n = length(t);
filtered_signal = zeros(1, n);
P = 0.4;       % Anfangswert der Fehlerkovarianz
Q = 0.01;      % Prozessrauschen
R = 0.5;      % Messrauschen

% P = 2;         % Anfangswert der Fehlerkovarianz
% Q = 0.01;      % Prozessrauschen
% R = 0.25;      % Messrauschen


x_est = 22;    % Initiale Schätzung des Zustands

% Platzhalter für verschlechterte Signale, gefilterte Signale und Health-Index
all_degraded_signals = zeros(days, n); % Verschlechterte Signale für alle Tage
all_filtered_signals = zeros(days, n); % Gefilterte Signale für alle Tage
mnqe_values = zeros(1, days); % MNQE-Werte pro Tag
all_health_indices = zeros(days, 1); % Health-Index für jeden Tag und Zeitpunkt

for day = 1:days
    % Verschlechterung nur ab dem zweiten Tag
    if day > 1
        degradation_factor = 0.1 * (day - 1); % Erhöhungsfaktor ab Tag 2
        all_degraded_signals(day, :) = measured_signal + degradation_factor * cos(2 * pi * t / 24);
    else
        all_degraded_signals(day, :) = measured_signal; % Keine Verschlechterung am ersten Tag
    end

    % Kalman-Filter erneut anwenden für das verschlechterte Signal
    for k = 1:n
        x_pred = x_est;
        P_pred = P + Q;
        K = P_pred / (P_pred + R);
        x_est = x_pred + K * (all_degraded_signals(day, k) - x_pred);
        P = (1 - K) * P_pred;
        filtered_signal(k) = x_est;

        % Health-Index für jeden Zeitpunkt berechnen
        mnqe = mean(((filtered_signal(1:k) - ref_signal(1:k)) / std(ref_signal(1:k))).^2);
        all_health_indices(day, end) = 1 / (1 + mnqe);
    end

    % Gefiltertes Signal speichern
    all_filtered_signals(day, :) = filtered_signal;

    % MNQE für diesen Tag berechnen
    mnqe_values(day) = mean(((filtered_signal - ref_signal) / std(ref_signal)).^2);
end

Meas_errorSignal=controlled_signal-measured_signal;
Est_errorSignal=controlled_signal-all_filtered_signals(analyzedDay, :);


MeasErrCov = sum(Meas_errorSignal.*Meas_errorSignal)/length(Meas_errorSignal);
EstErrCov = sum(Est_errorSignal.*Est_errorSignal)/length(Est_errorSignal);

% 3. Ergebnisse visualisieren

figure(1)
plot(t, ref_signal,'color', [.5 .5 .5], 'LineWidth', 0.75); hold on;
plot(t, controlled_signal, 'g-', 'LineWidth', 1.2);
legend('Soll-Temperatur', 'Ist-Temperatur');
xlabel('Zeit [h]');
ylabel('Temperatur [°C]');
title('Soll-/Ist-Temperatur Vergleich');
grid on;
ylim([18 26])


figure(2)
plot(t, ref_signal,'color', [.5 .5 .5], 'LineWidth', 0.75); hold on;
plot(t, controlled_signal, 'g-', 'LineWidth', 1.2);
plot(t, all_degraded_signals(analyzedDay, :), 'r-', 'LineWidth', 1.5);
legend('Soll-Temperatur','Ist-Temperatur', 'Messsignal mit Rauschen');
xlabel('Zeit [h]');
ylabel('Temperatur [°C]');
title(['Signal mit Noise (Tag ',num2str(analyzedDay),')']);
grid on;

figure(3)
plot(t, ref_signal,'color', [.5 .5 .5], 'LineWidth', 0.75); hold on;
plot(t, controlled_signal, 'g-', 'LineWidth', 1.2);
plot(t, all_degraded_signals(analyzedDay, :), 'r-', 'LineWidth', 0.75);
plot(t, all_filtered_signals(analyzedDay, :), 'b-', 'LineWidth', 1.5);
legend('Soll-Temperatur','Ist-Temperatur', 'Messsignal mit Rauschen', 'Gefiltertes Signal ');
legend('Soll-Temperatur','Gefiltertes Signal ');
xlabel('Zeit [h]');
ylabel('Temperatur [°C]');
title(['Signalfilterung mit Kalman-Filter (Tag ',num2str(analyzedDay),')']);
grid on;



figure(4)
plot(1:days, all_health_indices, 'm-o', 'LineWidth', 1.5);
hold on
plot(analyzedDay, all_health_indices(analyzedDay),'bo','MarkerFaceColor','b');
plot([0,days],[failureLimit,failureLimit],'--k', 'LineWidth', 1.5)
legend('Health-Index', ['Health-Index (Tag ' ,num2str(analyzedDay),')'],'Fehler-Schwellwert');
xlabel('Tage');
ylabel('Health-Index');
title(['Health-Index über die Lebensdauer der Anlage über ',num2str(days), ' Tage']);
grid on;

% Ergebnis-Anzeige
fprintf('Der berechnete EstErrCov beträgt: %.4f\n', EstErrCov);


