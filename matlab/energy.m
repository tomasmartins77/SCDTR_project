values = load("energy.csv") ;

hold on
plot((values(:,6)),values(:,1))
plot((values(:,6)),values(:,2), Color="green")


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([0 32])
xlim([7.02 42.9])
yyaxis right
plot((values(:,6)),values(:,4), LineWidth=1.5)
ylim([0 100])
xline(17.21, Color="r")

ylabel("Energy consumption ($\mu$J)", 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

legend( "y(t)", "r(t)",  "e(t)", 'Interpreter', 'latex')

grid on