load com6l.csv
load com5l.csv

plot((com6l(:,5)/1000000) ,com6l(:,1))
hold on
%plot((com6l(:,5)/1000000) -11.7,com6l(:,2))

plot((com5l(:,5)/1000000)-7.95,com5l(:,4))

ylabel("Luminance (Lux)")
xlabel("Time (s)")
ylim([-1 18])
xlim([0 20])
%xlim([0 7])
yyaxis right
plot((com6l(:,5)/1000000) ,com6l(:,3))
ylim([0 1])
ylabel("Duty cycle")
legend("y(t)", "external light", "d(t)")

grid on
