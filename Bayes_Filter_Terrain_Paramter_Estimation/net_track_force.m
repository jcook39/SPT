function [F_TNet, F, Rc, tauRes, sinkageM] = net_track_force(terrainHypothesis, nConstantMT865 , slip)

% Track Forces model form Theory of Ground Vehicles Wong 1st Ed. , pg 111-113 ex 2.1

% ----------- Unpack Needed nConstantMT865 Structure Paramters ------------
trackWidthM = nConstantMT865.trackWidthM;
trackLengthM = nConstantMT865.trackLengthM;
trackAreaM2 = nConstantMT865.trackAreaM2;
weightTractorN = nConstantMT865.weightTractorN;
normalForceTrackN = nConstantMT865.normalForceTrackN;
rollingRadiusM = nConstantMT865.rollingRadiusM;
trackDamp = nConstantMT865.trackDamp;


% -------------------- Unpack Terrain Hypothesis --------------------------
cohesion = terrainHypothesis(1,1);
frictionAngle = terrainHypothesis(2,1);
n = terrainHypothesis(3,1);
keq = terrainHypothesis(4,1);
K = terrainHypothesis(5,1);
S = terrainHypothesis(6,1);

% ------------------- Longitdunal Resistance Forces -----------------------

% Sinkage Calculation for Each Track
sinkageStaticM = calculate_sinkage(weightTractorN, trackAreaM2, keq, n);
slipSinkageSlope = S*sinkageStaticM;
sinkageM = sinkageStaticM + slipSinkageSlope*slip;

% Compaction Resistance
Rc = 2*trackWidthM*(keq).*((sinkageM.^(n+1))./(n+1));

% -------------------------- Traction Forces ------------------------------

% Calculate Traction Effort for Each Track;
maxF = 2* (trackAreaM2*cohesion + (normalForceTrackN/1000)*tand(frictionAngle) );
F = maxF.*(1 - ((K./(slip.*trackLengthM)).*(1 - exp((-slip.*trackLengthM)./K))));

% ------------ Convert Longitudinal Forces to kN from N -------------------
F = F*1000;
Rc = Rc*1000;
F_TNet = F - Rc;
tauRes = F*rollingRadiusM;

end


function sinkage = calculate_sinkage(weightTractorN, trackAreaM2 , keq , n)
    sinkage = ( ((weightTractorN/1000)/(2*trackAreaM2)) / (keq) )^(1/n);
end