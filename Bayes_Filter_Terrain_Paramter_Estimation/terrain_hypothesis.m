function structRBE = terrain_hypothesis(covariance, nConstantMT865, nTimeStep)

% --------------- Unpack Necessary Tractor Parameters ---------------------
trackWidthM = nConstantMT865.trackWidthM;

% ---------------------- Terrain Hypotheses -------------------------------
% terrainHypothesis is a matrix where each parameter vector in a single hypothesis
% is a column vector. Here every column vector corresponds to a different hypothesis.
%
% Each hypothesis consists of the following column vector:
%  [cohesion(kPa) frictionAngle(Deg) n(pressure-sinkage) keq K S(slip-sinkage coeff) staticSinkage].'
%  [c phi n keq K S].'

% Hypthesis 1 (Based on Lever Paper "Solar Power for an Antarctic Rover")
terrainHypothesisOne(1,1) = 6;
terrainHypothesisOne(2,1) = 20;
terrainHypothesisOne(3,1) = 1;
terrainHypothesisOne(4,1) = 500;
terrainHypothesisOne(5,1) = 1.5;     % Estimated (0.5)
terrainHypothesisOne(6,1) = 0.6/33;     % To be calculated

% Hypothesis 2 (Theory of Ground Vehicles Wong 3rd ed pg 136 Table 2.3) -
% Snow (U.S.)
terrainHypothesisTwo(1,1) = 1.03;
terrainHypothesisTwo(2,1) = 19.7;
terrainHypothesisTwo(3,1) = 1.6;
terrainHypothesisTwo(4,1) = 4.37/trackWidthM + 196.72;
terrainHypothesisTwo(5,1) = 2;   % Estimated
terrainHypothesisTwo(6,1) = 0.6/33;     % To be calculated

% Hypothesis 3 (Theory of Ground Vehicles Wong 3rd ed pg 136 Table 2.3) -
% Snow (Harrison)
terrainHypothesisThree(1,1) = 0.62;
terrainHypothesisThree(2,1) = 23.2;
terrainHypothesisThree(3,1) = 1.6;
terrainHypothesisThree(4,1) = 2.49/trackWidthM + 245.90;
terrainHypothesisThree(5,1) = 3; % Estimated
terrainHypothesisThree(6,1) = 0.6/33;     % To be calculated

% Hypothesis 4 (Theory of Ground Vehicles Wong 3rd ed pg 136 Table 2.3) -
% Snow (Sweden)
terrainHypothesisFour(1,1) = 6;
terrainHypothesisFour(2,1) = 20.7;
terrainHypothesisFour(3,1) = 1.44;
terrainHypothesisFour(4,1) = (10.55/trackWidthM + 66.08) + 250;
terrainHypothesisFour(5,1) = 2; 
terrainHypothesisFour(6,1) = 0.6/33;     % To be calculated

% Hypothesis 5 (Terramechanics and Off-Road Vehicle ENgineering Wong pg 198 Table 8.6) -
% Snow (Petawawa A & B)
terrainHypothesisFive(1,1) = 0.12;
terrainHypothesisFive(2,1) = 16.4;
terrainHypothesisFive(3,1) = 1.3;
terrainHypothesisFive(4,1) = 200;
terrainHypothesisFive(5,1) = 1.5;
terrainHypothesisFive(6,1) = 0.6/33;     % To be calculated

terrainHypotheses = [terrainHypothesisOne terrainHypothesisTwo terrainHypothesisThree...
    terrainHypothesisFour terrainHypothesisFive];

% -------------------------------------------------------------------------

% ------------------------- Terrain Hypotheses New ------------------------

structRBE.slipVectorBayes = linspace(0.01, 100, 101);

[terrainHypotheses, peakTractionMatHypotheses] = build_terrain_hypotheses(nConstantMT865, structRBE.slipVectorBayes);
nHypotheses = size(terrainHypotheses,2);

structRBE.terrainHypotheses = terrainHypotheses;
structRBE.terrainProbability = zeros(nHypotheses, nTimeStep);
structRBE.terrainProbability(:,1) = 1/nHypotheses;
structRBE.nHypotheses = nHypotheses;

structRBE.covarianceMatrix = diag(covariance);
structRBE.lowProbThreshold = 1E-3;

structRBE.peakTractionMatHypotheses = peakTractionMatHypotheses;

end

function [terrainMat, pkTractionMat] = build_terrain_hypotheses(nConstantMT865, slip )

c = 1:1:8;
phi = 20;
n = 1;
keq = 100:100:500;
K = 0.5:1:3.5;
S = 0.6/33;

nCohesion = numel(c);
nkeq = numel(keq);
nK = numel(K);
nCombination = nK*nkeq*nCohesion;
terrainMat = zeros(6,nCombination);
pkTractionMat = zeros(4,nCombination); % by column = [netTractionNoLoad, peakSlipNoLoad, netTractionLoad, peakSlipLoad].';
fprintf(' Number of Hypotheses = %f \n', nCombination)

terrainMatIndex = 1;
for i = 1:nCohesion
   for j = 1:nK
       for h = 1:nkeq
           terrainMat(:,terrainMatIndex) = [c(i) phi n keq(h) K(j) S].';
           [netTractionNoLoad, peakSlipNoLoad, netTractionLoad, peakSlipLoad] ...
               = peak_traction(nConstantMT865, terrainMat(:,terrainMatIndex), slip, 'MaxTraction');
           pkTractionMat(:,terrainMatIndex) = [netTractionNoLoad  peakSlipNoLoad  netTractionLoad  peakSlipLoad].'; 
           terrainMatIndex = terrainMatIndex + 1;
       end
   end
end

end



