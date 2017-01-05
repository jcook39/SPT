function [netTractionNoLoad, peakSlipNoLoad, netTractionLoad, peakSlipLoad, tauResPk] = peak_traction(nConstantMT865, terrainHypothesis, slip, tractionString)

% ------------------ Unpack Needed Paramters ------------------------------
Rs = nConstantMT865.RsledN;
mS = nConstantMT865.massSledKG;
mT = nConstantMT865.massTractorKG;

% ------ Compute Traction and Drawbar forces for all slip values ----------
[F_TNet, F, Rc, tauRes, ~] = net_track_force(terrainHypothesis, nConstantMT865 , slip);
DP = ( ((F - Rc)./mT) + (Rs./mS) ) .* ( (mT*mS)/(mT+mS) );

netTractionNoLoadAll = F_TNet;
netTractionLoadAll = F_TNet - DP;

netTractionNoLoadMax = max(netTractionNoLoadAll);
netTractionLoadMax = max(netTractionLoadAll);

indexNetTractionNoLoadMax = ( netTractionNoLoadAll == netTractionNoLoadMax );
indexNetTractionLoadMax = ( netTractionLoadAll == netTractionLoadMax );

tauResPk = tauRes(indexNetTractionNoLoadMax);

peakSlipNoLoad = slip(indexNetTractionNoLoadMax);
peakSlipLoad = slip(indexNetTractionLoadMax);

if strcmp(tractionString, 'AllTraction') % "All" gives forces for slip = 0.01:100
    netTractionNoLoad = netTractionNoLoadAll;
    netTractionLoad = netTractionLoadAll;
elseif strcmp(tractionString, 'MaxTraction')% Gives single value at peak slip
    netTractionNoLoad = netTractionNoLoadMax;
    netTractionLoad = netTractionLoadMax;
end

end