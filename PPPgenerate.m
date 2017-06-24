R1 = 3000; 
R2 = 200; 
R3 = 50;
NoofAPS =  poissrnd(100);
NoofNODES = poissrnd(2) + 1; %Atleast one node associated with AP,
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

% polar(angle,dist,'b.');
figure
scatter(APx,APy,'r.');
hold on
scatter(NODEx, NODEy, 'b.')
%figure
%polar(angleAP,distAP,'r.');
%hold on
%polar(angleNODE,distNODE,'b.');
