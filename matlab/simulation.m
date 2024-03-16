s = tf('s');

m = -0.9;
r = 30;
vcc = 3.3;
R = 10000;
C = 10e-6;
b = log10(225000) - m;
LDR = power(10, (log10(r) * m + b));
vss =  vcc * R / (LDR + R);

tau = (LDR * R) / (LDR + R) * C;
Ti = tau;
G = 34.085968;
H = vss/r;
K = 0.3;
b = 1/(G * K * H);

figure;
hold on

%values_1000 = load("values_1000.csv");


sys = ((b*Ti*s + 1)*(s*tau + 1))/((s*tau+1)*(Ti*s/(G*H*K)) + Ti*s + 1);

 %Time vector
t = linspace(0, 5, 1000); % Adjust the time vector according to your needs

% Generate a reference input (e.g., a step input from 0 to 20)
reference_input = 3*(t<1)+ r * (t >= 1); % Step input from 0 to 20

% Simulate the system response
output = lsim(sys, reference_input, t);

% Plot the results

plot(t, reference_input, 'b', 'LineWidth', 1.5, Color="#77AC30");
%plot((values_1000(:,5))-4.02,values_1000(:,1), linewidth = 1.5)
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
xlim([0.8 2])
ylim([0 32])
set(gca,'TickLabelInterpreter','latex')
grid on
plot(t, output, 'r', 'LineWidth', 1.5, Color="#0072BD");
legend("$r(t)$", "$yreal(t)$", "$ysim(t)$", 'Interpreter', 'latex')
