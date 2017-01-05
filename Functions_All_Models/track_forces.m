function [Fl,Fr,RL,RR,RlLF,RlRF,RlLR,RlRR,slipLeft,slipRight] = track_forces(state, MT865, nConstantMT865)

%Track Forces model form Theory of Ground Vehicles Wong 1st Ed. , pg 111-113 ex 2.1

%% Unpack Needed nConstantMT865 Structure Paramters
trackWidthM = nConstantMT865.trackWidthM;
trackLengthM = nConstantMT865.trackLengthM;
trackAreaM2 = nConstantMT865.trackAreaM2;
weightTractorN = nConstantMT865.weightTractorN;
normalForceTrackN = nConstantMT865.normalForceTrackN;
bT = nConstantMT865.trackCG2trackCG;
r = nConstantMT865.rollingRadiusM;
%% Unpack Needed State Variables
vx = state(6);
vy = state(7);
thetaDot = state(8);
wl = state(11);
wr = state(12);
%% Declare Constant Terrain Parameters and Unpack Spatial Ones
n = 1;
kc = 10.55;     % (kN/m^(n+2))
kphi = 66.08;   % (kN/m^(n+2))
kSum = 800;     % (kN/m^(n+3))

gammas = 3;     % kN/M3
%ul = 0.04;         % Lateral Coefficient of Turning
Ncp = 13;       %From figure 2.19, Theory of Ground Vehicles, Wong
Ngammap = 3;

% cohesionLeft = mean([MT865.terrainLeftFront(1) MT865.terrainLeftRear(1)]);
% frictionAngleLeft = mean([MT865.terrainLeftFront(2) MT865.terrainLeftRear(2)]);
% KLeft = mean([MT865.terrainLeftFront(3) MT865.terrainLeftRear(3)]);
% 
% cohesionRight = mean([MT865.terrainRightFront(1) MT865.terrainRightRear(1)]);
% frictionAngleRight = mean([MT865.terrainRightFront(2) MT865.terrainRightRear(2)]);
% KRight = mean([MT865.terrainRightFront(3) MT865.terrainRightRear(3)]);

cohesionLeft = ( MT865.terrainLeftFront(1) + MT865.terrainLeftRear(1) )/2;
frictionAngleLeft = ( MT865.terrainLeftFront(2) + MT865.terrainLeftRear(2) )/2;
KLeft = ( MT865.terrainLeftFront(3) + MT865.terrainLeftRear(3) )/2;

cohesionRight = ( MT865.terrainRightFront(1) + MT865.terrainRightRear(1) )/2;
frictionAngleRight = ( MT865.terrainRightFront(2) + MT865.terrainRightRear(2) )/2;
KRight = ( MT865.terrainRightFront(3) + MT865.terrainRightRear(3) )/2;


%% Longitdunal Resistance Forces 

% Sinkage Calculation for Each Track
sinkageLeft = (((normalForceTrackN/1000)./(trackAreaM2))./(kSum)).^(1/n);
sinkageRight = (((normalForceTrackN/1000)./(trackAreaM2))./(kSum)).^(1/n);
sinkageLeft = 0.1;
sinkageRight = 0.1;

% Compaction Resistance
RcL = trackWidthM*(kSum).*((sinkageLeft.^(n+1))./(n+1));
RcR = trackWidthM*(kSum).*((sinkageRight.^(n+1))./(n+1));

% Bulldozing Resistance
    % Left Track
    tandFrictionAnglePrimeLeft= (2/3)*tand(frictionAngleLeft);                      %Theory of Ground Vehicles, Wong pg 97
    FrictionAnglePrimeLeft = atand(tandFrictionAnglePrimeLeft);
    Kpcp = (Ncp - tandFrictionAnglePrimeLeft)*((cosd(FrictionAnglePrimeLeft))^2);
    Kpgammap = ((2*Ngammap/tandFrictionAnglePrimeLeft) + 1)*((cosd(FrictionAnglePrimeLeft))^2);
    RbL = trackWidthM*((0.67*cohesionLeft*sinkageLeft*Kpcp) + (0.5*(sinkageLeft^2)*gammas*Kpgammap)); % (kN)
    % Right Track
    tandFrictionAnglePrimeRight= (2/3)*tand(frictionAngleRight);                      %Theory of Ground Vehicles, Wong pg 97
    FrictionAnglePrimeRight = atand(tandFrictionAnglePrimeRight);
    Kpcp = (Ncp - tandFrictionAnglePrimeRight)*((cosd(FrictionAnglePrimeRight))^2);
    Kpgammap = ((2*Ngammap/tandFrictionAnglePrimeRight) + 1)*((cosd(FrictionAnglePrimeRight))^2);
    RbR = trackWidthM*((0.67*cohesionRight*sinkageRight*Kpcp) + (0.5*(sinkageRight^2)*gammas*Kpgammap)); % (kN)

% %Internal Resistance of the Running Gear Right and Left Track
W_tons = ((((weightTractorN/2)/1000)*224.8)/2204.6)/1000;  %Vehicle weight metric ton force
kmphL = (wl*r)*((60*60)/1000);                  %Convert m/s to km/hr
kmphR = (wr*r)*((60*60)/1000); 
RiL = 0.5*W_tons*(222+(3*(kmphL)));          % (kN) pg 184,198 Theory of Ground Vehicles, Wong
RiR = 0.5*W_tons*(222+(3*(kmphR)));     

% Total Resistsance at Each Track
RL = (RiL + RcL + RbL)*(sqrt(vx^2 + vy^2) > 0);
RR = (RiR + RcR + RbR)*(sqrt(vx^2 + vy^2) > 0);
%% Traction Forces

%Calculate Slip Ratios
vxLeft = vx - thetaDot*(bT/2);
vxRight = vx + thetaDot*(bT/2);

if (wl <= 0 ), slipLeft = 0;
else slipLeft = 1 - (vxLeft/(wl*r)); 
    if slipLeft < -1, slipLeft = -1; end
    if slipLeft > 1, slipLeft = 1; end
end
slipLeft = slipLeft*100 + 1E-10;

    
if (wr <= 0 ), slipRight = 0;
else slipRight = 1 - (vxRight/(wr*r)); 
    if slipRight < -1, slipRight = -1; end
    if slipRight > 1, slipRight = 1; end
end
slipRight = slipRight*100 + 1E-10;

% Calculate Traction Effort for Each Track;
maxFright = trackAreaM2*cohesionRight + (normalForceTrackN/1000)*tand(frictionAngleRight);
maxFleft = trackAreaM2*cohesionLeft + (normalForceTrackN/1000)*tand(frictionAngleLeft);
Fr = maxFright*(1 - ((KRight/(slipRight*trackLengthM))*(1 - exp((-slipRight*trackLengthM)/KRight))));
Fl = maxFleft*(1 - ((KLeft/(slipLeft*trackLengthM))*(1 - exp((-slipLeft*trackLengthM)/KLeft))));
%% Lateral Resistance Forces
    % Front Left
    vyFL = vy + (trackLengthM/4)*thetaDot;
    vxFL = vx - (bT/2)*thetaDot;
    % Front Right
    vyFR = vyFL;
    vxFR = vx + (bT/2)*thetaDot;
    % Rear Left
    vyRL = vy - (trackLengthM/4)*thetaDot;
    vxRL = vx - (bT/2)*thetaDot;
    % Rear Right
    vyRR = vyRL;
    vxRR = vx + (bT/2)*thetaDot;
    
if thetaDot ==0
    ul(1) = 0;
    ul(2) = 0;
    ul(3) = 0;
    ul(4) = 0;
else
    rowFL = abs( sqrt(vxFL^2 + vyFL^2)/thetaDot );
    rowFR = abs( sqrt(vxFR^2 + vyFR^2)/thetaDot );
    rowRL = abs( sqrt(vxRL^2 + vyRL^2)/thetaDot );
    rowRR = abs( sqrt(vxRR^2 + vyRR^2)/thetaDot );
    row = [rowFL rowFR rowRL rowRR].';
   % fprintf('row = %f %f %f %f \n',row(1), row(2) , row(3) , row(4))
    row(row > 1000) = 1000;
    row(row < 0.01) = 0.01;
    %ul = -0.0520*log(row) + 0.3604;
    ul = -0.0433*log(row) + 0.3004;
end

RlLF = -sign(vyFL)*ul(1)*(normalForceTrackN/2);
RlRF = -sign(vyFR)*ul(2)*(normalForceTrackN/2);
RlLR = -sign(vyRL)*ul(3)*(normalForceTrackN/2);
RlRR = -sign(vyRR)*ul(4)*(normalForceTrackN/2);
%% Convert Longitudinal Forces to kN from N 
Fr = Fr*1000*(Fr>0);
Fl = Fl*1000*(Fl>0);
RL = RL*1000;
RR = RR*1000;
end


