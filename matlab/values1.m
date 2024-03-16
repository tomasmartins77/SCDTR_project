values_01 = load("values_01.csv");

values_03 = load("values_03.csv");
values_12 = load("values_12.csv");

plot((values_03(400:1439,5))-4,values_03(400:1439,2)*0.0260)

hold on

%plot((values_12(400:1439,5))-4, values_12(400:1439,1))
%plot((values_03(400:1439,5))-4,values_03(400:1439,1), linewidth = 2)
%plot((values_01(400:1439,5))-4,values_01(400:1439,1))


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([0 33])
xlim([0 10.38])
%yyaxis right


plot((values_12(400:1439,5))-4,values_12(400:1439,4))
plot((values_03(400:1439,5))-4,values_03(400:1439,4), linewidth = 2)
plot((values_01(400:1439,5))-4,values_01(400:1439,4)) % "$r(t)$","$y_{K1.2}(t)$", "$y_{K0.3}(t)$", "$y_{K0.1}(t)$",
set(gca,'TickLabelInterpreter','latex')
ylim([0 1])
ylabel("Duty cycle", 'Interpreter', 'latex')
legend("$r_{dc}(t)$", "$d_{K1.2}(t)$", "$d_{K0.3}(t)$", "$d_{K0.1}(t)$", 'Interpreter', 'latex')

grid on