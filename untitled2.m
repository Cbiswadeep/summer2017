clear all
clc

R1 = 3000; 
R2 = 200; 
R3 = 50;
NoofAPS =  poissrnd(100)
NoofNODES = poissrnd(2) + 1 %Atleast one node associated with AP,
                            % code doesn't handle 0 nodes for an AP yet
distAP=[];angleAP=[];
    for i=1:NoofAPS
        dist_temp = R1*sqrt(rand);
        angle_temp = rand*(2*pi);
        for j=1:NoofNODES
            distAP = [distAP dist_temp];
            angleAP = [angleAP angle_temp];
        end
    end


angles=[];dists=[];

for j = 1:NoofNODES
    for i = 1:NoofAPS
        angles = [angles rand*(2*pi)];
        dists = [dists R2*sqrt(rand)];
    end
end

distNODE = sqrt(distAP.^2 + dists.^2 + 2*distAP.*dists.*cos(angles - angleAP));
angleNODE = angleAP + atan2(dists.*sin(angles - angleAP), distAP + dists.*cos(angles - angleAP));

APx=distAP.*cos(angleAP);
APy=distAP.*sin(angleAP);

NODEx=distNODE.*cos(angleNODE);
NODEy=distNODE.*sin(angleNODE);



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
        

        sensorLocales(:,1) = APx;
        sensorLocales(:,2) = APy;
        %sensorLocales
        %random
        %sensorLocales = randi([0 xrange],M,2);
        %linear below
        %sensorLocales = [12,10;17,10;22,10;27,10;32,10;37,10];
        %linear above
        %sensorLocales = [12,40;17,40;22,40;27,40;32,40;37,40];
        %vertical
        %sensorLocales = [18,35;18,25;18,15;32,35;32,25;32,15];
        sensorDist = sqrt((sensorLocales(1:M,1)-x0).^2 + (sensorLocales(1:M,2)-y0).^2);
        for i=1:T
         tau1(1:M,i) = sensorDist(1:M,:)./c + normrnd(mu,sigma,M,1); %toa
        end
        for i=2:M
         delta(i-1,:) = (tau1(i,:) - tau1(1,:)); %tdoa
        end
        %%% Search %%%
        minDist = 0;
        minSSE = 10000;
        yest = 100;
        xest = 100;
        sse = 0;


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


         if(sseFinal < minSSE) %minimize sum squared error
         minSSE = sseFinal;
         minDist = estTime*c;
         xest = x;
         yest = y;

         end
         sse=0;
         end
        end


        mseArray = (((sseArray)./T)./(M-1)).*c ;
        
        
        
       RMSE = sqrt((x0-xest)^2 + (y0 - yest)^2)
       figure
       % Root Mean Squared Error
       scatter(NoofAPS , RMSE , '*' , 'r')
       xlabel('Number of APS');
       ylabel('Root Mean Square Error');
      
       figure
       scatter(APx,APy,'r.');
       hold on
       scatter(NODEx, NODEy, 'b.')

        
        %convert to meters^2
       

%%% Plot %%%
%figure
%meshc(xArray, yArray, mseArray)
%xlabel('Y');
%ylabel('X');
%zlabel('Mean Square Error');

%qaz = minSSE * 0.9;
%zaq = minSSE / 0.9;
%ra = (qaz+ zaq)/2;
%P2 = [12 12];
%P1 = [xest yest];
%P0 = [18 12];
%ang = atan2(abs(det([P2-P0;P1-P0])),dot(P2-P0,P1-P0)); 

%figure
%L = ellipse((qaz+zaq),(zaq-qaz),ang,xest,yest,'r',300);
%rotate(L,[0 1 0],ang)
%hold on
%scatter(APx,APy,'r.');
%hold on
%scatter(xest,yest,'*','r')
%xlim([0 50])
%ylim([0 50])
%xlabel('X');
%ylabel('Y');
%hold off
%estimated location
xest
yest