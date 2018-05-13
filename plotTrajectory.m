close all
M = dlmread('trajectoryLog.txt',';');
N = dlmread('errorLog.txt');
P = dlmread('DistanceToIdeal.txt');
% O = dlmread('vectorErrorLog.txt', ';');
%plot the whole trajectory
plot3(M(:,1), M(:,2), -M(:,3));
figure 
startZ = M(1,1);
zDiff = -(M(:,1) - startZ);
disp(zDiff)
%plot the error on the altitude
plot(zDiff, M(:,2));
title('altitude error')
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
plot(M(:,1), -M(:,3), '-');

figure
plot(N)
hold on
plot(P)
title('error log')

% figure
% plot3(O(:,1), O(:,2), -O(:,3));
% title('Vector error log')

