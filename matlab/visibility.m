values = load("visibility.csv") ;

hold on
plot((values(:,6)),values(:,1))
plot((values(310:2607,6)),values(310:2607,2), Color="green")


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([0 32])
xlim([10 32.93])
yyaxis right
plot((values(:,6)),values(:,4), LineWidth=1.5)
ylim([0 2])
xline(22.59, Color="r")

ylabel("Visibility error (LUX)", 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

legend( "y(t)", "r(t)",  "v(t)", 'Interpreter', 'latex')

grid on