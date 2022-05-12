%Error Plot
Xerror = out.Xcords.Data - Xpos;
Yerror = out.Ycords.Data - Ypos;
figure(1)
hold on
grid on
plot(SimTimeVector, Xerror, 'r', 'LineWidth', 3);
plot(SimTimeVector, Yerror, 'k', 'LineWidth', 3);
xlabel('Time(seconds)');
ylabel('Error (cm)');
title('Error in X and Y position');
legend('X Error', 'Y Error');
