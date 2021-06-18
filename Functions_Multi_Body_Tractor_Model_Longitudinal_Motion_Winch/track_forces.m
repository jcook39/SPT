function [Fl,Fr,RL,RR,RlLF,RlRF,RlLR,RlRR,slipLeft,slipRight,sinkageLeftDynamic,sinkageRightDynamic] = track_forces(state, MT865, nConstantMT865)

%Track Forces model form Theory of Ground Vehicles Wong 1st Ed. , pg 111-113 ex 2.1

% ------------ Unpack Needed nConstantMT865 Structure Paramters -----------
trackWidthM = nConstantMT865.trackWidthM;
trackLengthM = nConstantMT865.trackLengthM;
trackAreaM2 = nConstantMT865.trackAreaM2;
weightTractorN = nConstantMT865.weightTractorN;
normalForceTrackN = nConstantMT865.normalForceTrackN;
bT = nConstantMT865.trackCG2trackCG;
r = nConstantMT865.rollingRadiusM;
% ---------------------- Unpack Needed State Variables --------------------
vx = state(4);
vy = state(5);
thetaDot = state(6);
wl = state(7);
wr = state(8);

% ---- Declare Constant Terrain Parameters and Unpack Spatial Ones --------
ul = 4;         % Lateral Coefficient of Turning

% Terrain Parameters Left and Right Track
cohesionLeft = mean([MT865.terrainLeftFront(1) MT865.terrainLeftRear(1)]);
frictionAngleLeft = mean([MT865.terrainLeftFront(2) MT865.terrainLeftRear(2)]);
KLeft = mean([MT865.terrainLeftFront(3) MT865.terrainLeftRear(3)]);
keqLeft = mean([MT865.terrainLeftFront(4) MT865.terrainLeftRear(4)]);
nLeft = mean([MT865.terrainLeftFront(5) MT865.terrainLeftRear(5)]);
SLeft = mean([MT865.terrainLeftFront(6) MT865.terrainLeftRear(6)]);

cohesionRight = mean([MT865.terrainRightFront(1) MT865.terrainRightRear(1)]);
frictionAngleRight = mean([MT865.terrainRightFront(2) MT865.terrainRightRear(2)]);
KRight = mean([MT865.terrainRightFront(3) MT865.terrainRightRear(3)]);
keqRight = mean([MT865.terrainRightFront(4) MT865.terrainRightRear(4)]);
nRight = mean([MT865.terrainRightFront(5) MT865.terrainRightRear(5)]);
SRight = mean([MT865.terrainRightFront(6) MT865.terrainRightRear(6)]);

% ---------------------- Calculate Slip Ratios ----------------------------
vxLeft = vx - thetaDot*(bT/2);
vxRight = vx + thetaDot*(bT/2);

if (wl <= 0 ), slipLeft = 0;
else slipLeft = 1 - (vxLeft/(wl*r)); 
    if slipLeft < 0, slipLeft = 0; end
    if slipLeft > 1, slipLeft = 1; end
end
slipLeft = slipLeft*100 + 1E-10;
    
if (wr <= 0 ), slipRight = 0;
else slipRight = 1 - (vxRight/(wr*r)); 
    if slipRight < 0, slipRight = 0; end
    if slipRight > 1, slipRight = 1; end
end
slipRight = slipRight*100 + 1E-10;

%fprintf('slipLeft: %d , slipRight: %d \n',slipLeft,slipRight)

% --------------------- Longitdunal Resistance Forces ---------------------

% Sinkage Calculation for Each Track
sinkageLeftStatic = (((normalForceTrackN/1000)./(trackAreaM2))./(keqLeft)).^(1/nLeft);
sinkageRightStatic = (((normalForceTrackN/1000)./(trackAreaM2))./(keqRight)).^(1/nRight);

sinkageLeftDynamic = sinkageLeftStatic + (SLeft*sinkageLeftStatic)*slipLeft;
sinkageRightDynamic = sinkageRightStatic + (SRight*sinkageLeftStatic)*slipRight;

% Compaction Resistance
RcL = trackWidthM*(keqLeft).*((sinkageLeftDynamic.^(nLeft+1))./(nLeft+1));
RcR = trackWidthM*(keqRight).*((sinkageRightDynamic.^(nRight+1))./(nRight+1));

% %Internal Resistance of the Running Gear Right and Left Track
W_tons = ((((weightTractorN/2)/1000)*224.8)/2204.6)/1000;  %Vehicle weight metric ton force
kmphL = (wl*r)*((60*60)/1000);                  %Convert m/s to km/hr
kmphR = (wr*r)*((60*60)/1000); 
RiL = 0.5*W_tons*(222+(3*(kmphL)));          % (kN) pg 184,198 Theory of Ground Vehicles, Wong
RiR = 0.5*W_tons*(222+(3*(kmphR)));
RiL = 0;
RiR = 0;

% Total Resistsance at Each Track
RL = (RiL + RcL)*(sqrt(vx^2 + vy^2) > 0);
RR = (RiR + RcR)*(sqrt(vx^2 + vy^2) > 0);

% ----------------------- Traction Forces ---------------------------------

% Calculate Traction Effort for Each Track;
maxFright = trackAreaM2*cohesionRight + (normalForceTrackN/1000)*tand(frictionAngleRight);
maxFleft = trackAreaM2*cohesionLeft + (normalForceTrackN/1000)*tand(frictionAngleLeft);
Fr = maxFright*(1 - ((KRight/(slipRight*trackLengthM))*(1 - exp((-slipRight*trackLengthM)/KRight))));
Fl = maxFleft*(1 - ((KLeft/(slipLeft*trackLengthM))*(1 - exp((-slipLeft*trackLengthM)/KLeft))));

% ----------------------- Lateral Resistance Forces -----------------------
vyFL = vy + (trackLengthM/4)*thetaDot;
vyFR = vyFL;
vyRL = vy - (trackLengthM/4)*thetaDot;
vyRR = vyRL;

RlLF = -sign(vyFL)*ul*(normalForceTrackN/2);
RlRF = -sign(vyFR)*ul*(normalForceTrackN/2);
RlLR = -sign(vyRL)*ul*(normalForceTrackN/2);
RlRR = -sign(vyRR)*ul*(normalForceTrackN/2);

% ------------ Convert Longitudinal Forces to kN from N -------------------
Fr = Fr*1000;
Fl = Fl*1000;
RL = RL*1000;
RR = RR*1000;
end


