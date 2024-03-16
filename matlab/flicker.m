values = load("flickerfilter.csv") ;
values1 = load("flickernofilter.csv");

hold on
plot((values(:,6))-7.02,values(:,4))
plot((values1(:,6))-7.02,values1(:,4))

xlabel("Time (s)", 'Interpreter', 'latex')
ylabel("Flicker ($\mu s^-1$)", 'Interpreter', 'latex')

ylim([0 310])
xlim([0 52.94])

set(gca,'TickLabelInterpreter','latex')

legend( "With filter", "No filter", 'Interpreter', 'latex')

grid on