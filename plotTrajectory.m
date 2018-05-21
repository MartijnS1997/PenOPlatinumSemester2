close all
M = dlmread('trajectoryLog.txt',';');
N = dlmread('errorLog.txt');
% P = dlmread('DistanceToIdeal.txt');
O = dlmread('vectorErrorLog.txt', ';');
%plot the whole trajectory
plot3(M(:,1), M(:,2), -M(:,3));
figure 

plot(N)

figure
plot(O(:,3),O(:,1))
figure
plot(O(:,3),O(:,2))

