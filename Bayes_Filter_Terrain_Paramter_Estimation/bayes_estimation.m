function [ structRBE ] = bayes_estimation(structRBE, structDTKF, nConstantMT865, time, timeStepNo)
% This function implements a Bayes Estimator where the inputs are the force
% vector

% ----------------- Unpack Constant Tractor Parameters --------------------
rollingRadiusM = nConstantMT865.rollingRadiusM;
normalForceN = nConstantMT865.weightTractorN;

% ------------------- Unpack Estimated State from DTKF --------------------
estimatedState = structDTKF.xHatPlus(:,timeStepNo);
    vHat = estimatedState(1,1);
    omegaHat = estimatedState(2,1);
    forceVectorDTKF = [estimatedState(3,1) estimatedState(5,1)].';
    DB = estimatedState(7,1);
    
slipHat = structDTKF.slipHat(1,timeStepNo);
slipHatSmooth = structDTKF.slipHatSmooth(1,timeStepNo);

% ----------------------- Unpack structRBE --------------------------------
terrainHypotheses = structRBE.terrainHypotheses;
prior = structRBE.terrainProbability(:,timeStepNo-1);
nHypotheses = structRBE.nHypotheses;
covarianceMatrix = structRBE.covarianceMatrix;
lowProbThreshold = structRBE.lowProbThreshold;
normalizeString = structRBE.normalizeString;
slipVectorBayes = structRBE.slipVectorBayes;

% --------------------------  Bayes Estimator -----------------------------

liklihood = zeros(nHypotheses,1);
forceNormalizationFactor = [normalForceN rollingRadiusM*normalForceN].';
for k = 1:nHypotheses
    % Calculate Force Vectors
    [F_TNet, ~, ~, tauRes, ~] = net_track_force(terrainHypotheses(:,k), nConstantMT865 , slipHatSmooth);
    forceVectorHypothesis = [F_TNet tauRes].';
    
    % Compute the liklihood of the hypotheses
    if strcmp(normalizeString, 'normalize')
        forceVectorHypothesisNormalized = forceVectorHypothesis./forceNormalizationFactor;
        forceVectorDTKFNormalized = forceVectorDTKF./forceNormalizationFactor;
        liklihood(k,1) = mvnpdf(forceVectorDTKFNormalized.', forceVectorHypothesisNormalized.', covarianceMatrix);
    elseif strcmp(normalizeString, 'noNormalize')
        liklihood(k,1) = mvnpdf(forceVectorDTKF.', forceVectorHypothesis.', covarianceMatrix);
    end
end

normalization = sum(liklihood.*prior);
posteriorProbability = (liklihood.*prior)./normalization;

% -------------------------- Threshold Correction -------------------------

isLowProbThresholdViolation = (sum(posteriorProbability < lowProbThreshold)) >= 1;
if isLowProbThresholdViolation
% Check for entries below the probability threshold
    indexLowProbThresholdViolation = (posteriorProbability < lowProbThreshold);
    diffVector = lowProbThreshold - posteriorProbability(indexLowProbThresholdViolation);
    totalDiff = sum(diffVector);

% Correct Probabilities
    indexMaxPosteriorProbability = (posteriorProbability == max(posteriorProbability));
    posteriorProbability(indexMaxPosteriorProbability) = posteriorProbability(indexMaxPosteriorProbability) - totalDiff;
    posteriorProbability(indexLowProbThresholdViolation) = lowProbThreshold;
end

% -------------------------------------------------------------------------


% ----------------- Compute Peak Slip Point -------------------------------
parameterEstimate = terrainHypotheses*posteriorProbability;
[netTractionNoLoadEstimateMax, peakSlipNoLoad, ~, peakSlipLoad, ~] = ...
    peak_traction(nConstantMT865, parameterEstimate, slipVectorBayes, 'MaxTraction');
isSlipDiscrepancy = ( abs(peakSlipNoLoad - peakSlipLoad) > eps);
if isSlipDiscrepancy
    error('Error: peak slip discrepancy, check function peak_traction')
end
peakSlip = peakSlipNoLoad;

% ------------------- Package Recrusive Bayes Structure -------------------
structRBE.terrainProbability(:,timeStepNo) = posteriorProbability;
structRBE.parameterEstimate(:,timeStepNo) = parameterEstimate.';
structRBE.liklihood(:,timeStepNo) = liklihood;
structRBE.normalization(timeStepNo) = normalization;
structRBE.peakSlip(1,timeStepNo) = peakSlip;
structRBE.netTractionNoLoadEstimateMax(1,timeStepNo) = netTractionNoLoadEstimateMax;

end

