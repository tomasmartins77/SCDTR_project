values = load("bumpless.csv") ;

hold on
plot((values(:,5)),values(:,1))
plot((values(:,5)),values(:,2), Color="green")


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([14 29])
xlim([0 16.02])
yyaxis right
plot((values(:,5)),values(:,4))
xline(7.5, 'Color', 'red')
xline(2, "--", 'Color', 'red')
xline(5, "--", 'Color', 'red')
xline(13.24, "--", 'Color', 'red')
xline(9.77, "--", 'Color', 'red')
ylim([0 1])
ylabel("Duty cycle", 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

legend( "y(t)", "r(t)",  "d(t)", 'Interpreter', 'latex')

grid on