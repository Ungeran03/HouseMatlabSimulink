
clear all

P = 0.5;       % Anfangswert der Fehlerkovarianz
Q = 0.0;      % Prozessrauschen
R = 2;      % Messrauschen
x_est = 23;    % Initiale Schätzung des Zustands
y=33;          % Messwert mit Fehler
n=1;          % Anzahl der notwendigen Schätzungen


    for k = 1:n
        x_pred = x_est;                     %Schätzwert t-1  
        P_pred = P + Q;                     %Schätzfehler
        K = P_pred / (P_pred + R);          %Kalmanverstärkung
        x_est = x_pred + K * (y - x_pred);  %Schätzwert
        P = (1 - K) * P_pred;               %Korrektur des Schätzfehlers
        x_out(k)=x_est;

    end

    plot(x_out);