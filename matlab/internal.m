values = load("external1.csv") ;
plot((values(:,5)),values(:,3),'LineWidth',1.4, 'color', "green")

hold on
plot((values(:,5)),values(:,1), Color="#0072BD")
plot((values(:,5)),values(:,2), "--")


ylabel("Luminance (Lux)", 'Interpreter', 'latex')
xlabel("Time (s)", 'Interpreter', 'latex')
ylim([0 35])
xlim([0 14])
yyaxis right
plot((values(:,5)),values(:,4))
ylim([0 1])
ylabel("Duty cycle", 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')
legend("o(t)", "y(t)", "r(t)",  "d(t)", 'Interpreter', 'latex')

grid on
