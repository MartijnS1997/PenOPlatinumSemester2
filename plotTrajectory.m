close all
% M = dlmread('trajectoryLog.txt',';');
N = dlmread('errorLog.txt');
% P = dlmread('DistanceToIdeal.txt');
% O = dlmread('vectorErrorLog.txt', ';');
%plot the whole trajectory
% plot3(M(:,1), M(:,2), -M(:,3));
% figure 
% startZ = M(1,1);
% zDiff = -(M(:,1) - startZ);
% disp(zDiff)
% %plot the error on the altitude
% plot(zDiff, M(:,2));
% title('altitude error')
% figure
%plot the error on the circle
% xCenter = mean(M(:,1));
% zCenter = mean(M(:,3));
% theta = 0 : 0.01 : 2*pi;
% radius = 1000;
% x = radius * cos(theta) + xCenter;
% z= radius * sin(theta) + zCenter;
% plot(x, z, '--');
% hold on
% title('flight path top view')
% plot(M(:,1), -M(:,3), '-');
alpha = 0.99;
% X = linspace(0,2*pi, 1000);
% H = alpha ./ (1-exp(i.*X).*(1-alpha));
% plot(abs(H))
% disp(H)

figure
L = fft(N);
plot(abs(L(2:end)))
%ldf
% l = floor(length(L)/20);
% L2 = zeros(length(L));
% L2(1:l) = L(1:l);
N = length(L);
n = linspace(0,N,N);
H = alpha ./ (1-exp((2*pi*1i.*n)./N).*(1-alpha));
plot(2*n/N, abs(H))
L2 = H.*L;
N2 =ifft(L2);
figure
plot(N)
hold on
%plot(abs(N2))
% figure
% plot(abs(L))
% hold on
% plot(abs(L2))
