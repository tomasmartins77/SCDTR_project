values = load("jitter.csv") ;

hold on
bar((values(:,6))-7.04,values(:,5))

xlabel("Time (s)", 'Interpreter', 'latex')
ylabel("Jitter ($\mu s$)", 'Interpreter', 'latex')

ylim([0 8.2])
%xlim([0 52.98])

set(gca,'TickLabelInterpreter','latex')

grid on