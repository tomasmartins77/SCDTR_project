values = load("retas.csv");

x = values(1:17,1);
y = values(1:17,2);

x1 = values(18:34,1);
y1 = values(18:34,2);

x2 = values(35:51,1);
y2 = values(35:51,2);

b = x\y;
ycalc = b*x;

b1 = x1\y1;
ycalc1 = b1*x1;


b2 = x2\y2;
ycalc2 = b2*x2;

scatter(x, y, LineWidth=1.2, MarkerEdgeColor="#0072BD")
hold on
plot(x, ycalc, LineWidth=1.2, Color="#0072BD")

scatter(x1, y1, LineWidth=1.2, MarkerEdgeColor="#D95319")
plot(x, ycalc1, LineWidth=1.2, Color="#D95319")

scatter(x2, y2, LineWidth=1.2, MarkerEdgeColor=	"#A2142F")
plot(x, ycalc2, LineWidth=1.2, Color=	"#A2142F")


legend("m = -0.7", "", "m = -0.8", "", "m = -0.9", "", 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')
xlabel("Duty Cycle", 'Interpreter', 'latex')
ylabel("Lux", 'Interpreter', 'latex')
grid on