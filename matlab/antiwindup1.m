values = load("antiwindup.csv") ;

hold on
plot((values(:,5)),values(:,1))
plot((values(:,5)),values(:,2), "--")


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([0 25])
xlim([0 27.25])
yyaxis right
plot((values(:,5)),values(:,4))
xline(8, 'Color', 'red', 'LineWidth',1.2)
set(gca,'TickLabelInterpreter','latex')
ylim([0 1])
ylabel("Duty cycle", 'Interpreter', 'latex')
legend("y(t)", "r(t)",  "d(t)", 'Interpreter', 'latex')

grid on