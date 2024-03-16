values_01 = load("tau1.csv");

values_03 = load("tau06.csv");
values_12 = load("tau03.csv");

hold on

plot((values_12(:,3))-2.09, values_12(:,1))
plot((values_03(:,3))-2.09,values_03(:,1))
plot((values_01(:,3))-1.49,values_01(:,1))


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([0 36])
xlim([0 2.9])
set(gca,'TickLabelInterpreter','latex')

ylabel("Illuminance (LUX)", 'Interpreter', 'latex')
legend("$d_{0.3}(t)$","$d_{0.6}(t)$", "$d_{1}(t)$", 'Interpreter', 'latex')

grid on