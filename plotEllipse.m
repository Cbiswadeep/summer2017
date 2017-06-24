clc
clear all
%%% Constants %%%
T = 200; %shots
c = 343.2; %speed of sound
M = 6; %number of sensors
mu = 0;
sigma = .01;
xrange = 50.000;
yrange = 50.000;
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
%random
%sensorLocales = randi([0 xrange],M,2);
%linear below
%sensorLocales = [12,10;22,10;15,15;15,5;10,5;5,15];
%linear above
%sensorLocales = [12,40;17,40;22,40;27,40;32,40;37,40];
%vertical
%sensorLocales = [18,35;18,25;18,15;32,35;32,25;32,15]
sensorDist = sqrt((sensorLocales(1:M,1)-x0).^2 + (sensorLocales(1:M,2)-y0).^2 );
sensorDist1 = sqrt((sensorLocales(1:M,3)-x0).^2 + (sensorLocales(1:M,4)-y0).^2 );

for i=1:T
 tau1(1:M,i) = sensorDist(1:M,:)./c + normrnd(mu,sigma,M,1); %toa
end

for i=1:T
 tau2(1:M,i) = sensorDist1(1:M,:)./c + normrnd(mu,sigma,M,1); %toa
end

for i=2:M
 delta(i-1,:) = (tau1(i,:) - tau1(1,:)); %tdoa
end

for i=2:M
 delta1(i-1,:) = (tau2(i,:) - tau2(1,:)); %tdoa
end
%%% Search %%%
minDist = 0.000;
minSSE1 = 10000.000;
minSSE2 = 10000.000;
yest1 = 100.000;
xest1 = 100.000;
yest2 = 100.000;
xest2 = 100.000;
sse = 0.00;
sse1 = 0.00;
for x = 0:xrange
 for y = 0:yrange
 searchDist = sqrt((sensorLocales(1:M,1)-x).^2 + (sensorLocales(1:M,2)-y).^2);
 estTime = (searchDist)/c;
 for i=2:M
 deltaEst(i-1,:) = estTime(i) - estTime(1);
 end
 for i=1:T %calculate errors
 ssMsmts = sum((delta(1:M-1,i) - deltaEst(1:M-1)).^2);
 sse = ssMsmts + sse;
 end

 sseFinal = sse;
 sseArray(x+1,y+1) = sseFinal;
 xArray(1:yrange+1,x+1) = x;
 yArray(y+1,1:xrange+1) = y;


 if(sseFinal < minSSE1) %minimize sum squared error
 minSSE1 = sseFinal;
 minDist = estTime*c;
 xest1 = x;
 yest1 = y;
 end
 sse=0;
 end
end
xest1
yest1

for x = 0:xrange
 for y = 0:yrange
 searchDist = sqrt((sensorLocales(1:M,3)-x).^2 + (sensorLocales(1:M,4)-y).^2);
 estTime = (searchDist)/c;
 for i=2:M
 deltaEst(i-1,:) = estTime(i) - estTime(1);
 end
 for i=1:T %calculate errors
 ssMsmts = sum((delta(1:M-1,i) - deltaEst(1:M-1)).^2);
 sse = ssMsmts + sse;
 end

 sseFinal = sse;
 sseArray1(x+1,y+1) = sseFinal;
 xArray1(1:yrange+1,x+1) = x;
 yArray1(y+1,1:xrange+1) = y;


 if(sseFinal < minSSE2) %minimize sum squared error
 minSSE2 = sseFinal;
 minDist = estTime*c;
 xest2 = x;
 yest2 = y;
 end
 sse=0;
 end
end

xest2
yest2





mseArray = (((sseArray)./T)./(M-1)).*c; %convert to meters^2
mseArray1 = (((sseArray1)./T)./(M-1)).*c;
%%% Plot %%%

figure
meshc((xArray+xArray1)/2, (yArray+yArray1)/2, (mseArray+mseArray1)/2)
xlabel('x');
ylabel('y');
zlabel('Mean Square Error');



qaz1 = minSSE1 * 0.9;
zaq1 = minSSE1 / 0.9;
ra1 = (qaz1+ zaq1)/2;
P21 = [12 12];
P11 = [xest1 yest1];
P01 = [18 12];
ang1 = atan2(abs(det([P21-P01;P11-P01])),dot(P21-P01,P11-P01)); 

qaz2 = minSSE2 * 0.9;
zaq2 = minSSE2 / 0.9;
ra2 = (qaz2 + zaq2)/2;
P12 = [42 12];
P22 = [xest2 yest2];
P02 = [50 12];
ang2 = atan2(abs(det([P22-P02;P12-P02])),dot(P22-P02,P12-P02)); 



figure
L1 = ellipse((qaz1+zaq1),(zaq1-qaz1),ang1,xest1,yest1,'r',300);
rotate(L1,[0 1 0],ang1)

L2 = ellipse((zaq2 - qaz2),(zaq2+qaz2),ang2,xest2,yest2,'r',300);
rotate(L2,[0 1 0],ang2)

hold on
scatter(sensorLocales(:,1), sensorLocales(:,2))
hold on
scatter(sensorLocales(:,3), sensorLocales(:,4))
hold on
scatter((xest1+xest2)/2,(yest1+yest2)/2,'*','r')
xlim([0 50]);
ylim([0 50]);
xlabel('X');
ylabel('Y');
hold off
%estimated location
(xest1 + xest2)/2
(yest1 + yest2)/2