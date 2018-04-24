close all
M = dlmread('trajectoryLog.txt',';');
N = dlmread('errorLog.txt');
%plot the whole trajectory
plot3(M(:,1), M(:,2), M(:,3));
figure 
%plot the error on the altitude
title('altitude error')
plot(M(:,2));

figure
%plot the error on the circle
% xCenter = mean(M(:,1));
% zCenter = mean(M(:,3));
% theta = 0 : 0.01 : 2*pi;
% radius = 1000;
% x = radius * cos(theta) + xCenter;
% z= radius * sin(theta) + zCenter;
% plot(x, z, '--');
% hold on
title('flight path top view')
plot(M(:,1), M(:,3), '-');

figure
plot(N)
title('error log')


