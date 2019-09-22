figure
for itr = 1:1:Nitr
    plot([850:1:950],data.Driftcommand(850:950,itr))
    hold on
end
hold off
xlabel('actuator')
ylabel('DM Command')
title('Drift Command for each iteration')
legend('1','2','3','4','5')

d_drift = diff(data.Driftcommand,1,1);



figure
for itr = 1:1:Nitr-1
    plot([850:1:950],d_drift(850:950,itr))
    hold on
end
hold off
xlabel('actuator')
ylabel('Change in DM command')
title('Change in Drift Command between each iteration')
legend('2','3','4','5')