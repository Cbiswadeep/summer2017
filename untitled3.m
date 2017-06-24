close all
clear all
clc
%%% Constants %%%
T = 200; %shots
c = 343.2; %speed of sound
M = 6; %number of sensors
mu = 0;
sigma = .01;
xrange = 50;
yrange = 50;
%%% Sniper Location %%%
x0 = 25;
y0 = 25;
%%% Sensor Arrangement %%%
%circle
r = 7;
theta(1) = 0;
theta(2) = pi/3;
theta(3) = 2*pi/3;
theta(4) = pi;
theta(5) = 4*pi/3;
theta(6) = 5*pi/3;
sensorLocales(:,1) = 12 + r.*sin(theta);
sensorLocales(:,2) = 12 + r.*cos(theta);

sensorLocales(:,3) = 42 + r.*sin(theta);
sensorLocales(:,4) = 12 + r.*cos(theta);

sensorDist1 = sqrt((sensorLocales(1:M,1)-x0).^2 + (sensorLocales(1:M,2)-y0).^2);
sensorDist2 = sqrt((sensorLocales(1:M,3)-x0).^2 + (sensorLocales(1:M,4)-y0).^2);

for i=1:T
 tau1(1:M,i) = sensorDist1(1:M,:)./c + normrnd(mu,sigma,M,1); %toa
end
for i=1:T
 tau2(1:M,i) = sensorDist2(1:M,:)./c + normrnd(mu,sigma,M,1); %toa
end


for i=2:M
 delta1(i-1,:) = (tau1(i,:) - tau1(1,:)); %tdoa
end
for i=2:M
 delta2(i-1,:) = (tau2(i,:) - tau2(1,:)); %tdoa
end


%%% Search %%%
minDist1 = 0;
minSSE1 = 10000;
yest1 = 100;
xest1 = 100;
sse1 = 0;


for x = 0:xrange
 for y = 0:yrange
 searchDist1 = sqrt((sensorLocales(1:M,1)-x).^2 + (sensorLocales(1:M,2)-y).^2);
 estTime1 = (searchDist1)/c;
 for i=2:M
 deltaEst1(i-1,:) = estTime1(i) - estTime1(1);
 end
 for i=1:T %calculate errors
 ssMsmts1 = sum((delta(1:M-1,i) - deltaEst1(1:M-1)).^2);
 sse1 = ssMsmts1 + sse1;
 end

 sseFinal1 = sse1;
 sseArray1(x+1,y+1) = sseFinal1;
 xArray1(1:yrange+1,x+1) = x;
 yArray1(y+1,1:xrange+1) = y;


 if(sseFinal1 < minSSE1) %minimize sum squared error
 minSSE1 = sseFinal1;
 minDist1 = estTime*c;
 xest1 = x;
 yest1 = y;
 
 end
 sse1=0;
 end
end

xest1
yest1

minDist2 = 0;
minSSE2 = 10000;
yest2 = yest1;
xest2 = xest1;
sse2 = 0;


for x = xest1:xrange
 for y = yest1:yrange
 searchDist2 = sqrt((sensorLocales(1:M,3)-x).^2 + (sensorLocales(1:M,4)-y).^2);
 estTime2 = (searchDist2)/c;
 for i=2:M
 deltaEst2(i-1,:) = estTime(i) - estTime(1);
 end
 for i=1:T %calculate errors
 ssMsmts2 = sum((delta(1:M-1,i) - deltaEst1(1:M-1)).^2);
 sse2 = ssMsmts2 + sse2;
 end

 sseFinal2 = sse2;
 sseArray2(x+1,y+1) = sseFinal2;
 xArray2(1:yrange+1,x+1) = x;
 yArray2(y+1,1:xrange+1) = y;


 if(sseFinal2 < minSSE1) %minimize sum squared error
 minSSE2 = sseFinal1;
 minDist2 = estTime*c;
 xest2 = x;
 yest2 = y;
 
 end
 sse2=0;
 end
end

xest2
yest2





mseArray = (((sseArray1)./T)./(M-1)).*c; %convert to meters^2
%%% Plot %%%
figure
meshc(xArray1, yArray1, mseArray)
xlabel('Y');
ylabel('X');
zlabel('Mean Square Error');

qaz = minSSE1 * 0.9;
zaq = minSSE1 / 0.9;
ra = (qaz+ zaq)/2;
P2 = [12 12];
P1 = [xest1 yest1];
P0 = [18 12];
ang = atan2(abs(det([P2-P0;P1-P0])),dot(P2-P0,P1-P0)); 

figure
L = ellipse((qaz+zaq),(zaq-qaz),ang,xest1,yest1,'r',300);
rotate(L,[0 1 0],ang)
hold on
scatter(sensorLocales(:,1), sensorLocales(:,2))
hold on
scatter(xest1,yest1,'.','r')
xlim([0 50])
ylim([0 50])
xlabel('X');
ylabel('Y');
hold off
%estimated location
