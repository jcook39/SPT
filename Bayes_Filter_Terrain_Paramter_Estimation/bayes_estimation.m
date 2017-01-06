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
R_S = estimatedState(7,1);

slipHat = calculate_slip(vHat, omegaHat, nConstantMT865);

% ----------------------- Unpack structRBE --------------------------------
terrainHypotheses = structRBE.terrainHypotheses;
peakTractionMatHypotheses = structRBE.peakTractionMatHypotheses;
prior = structRBE.terrainProbability(:,timeStepNo-1);
nHypotheses = structRBE.nHypotheses;
covarianceMatrix = structRBE.covarianceMatrix;
lowProbThreshold = structRBE.lowProbThreshold;
normalizeString = structRBE.normalizeString;


% --------------------------  Bayes Estimator -----------------------------

liklihood = zeros(nHypotheses,1);
forceNormalizationFactor = [normalForceN rollingRadiusM*normalForceN].';
for k = 1:nHypotheses
    % Calculate Force Vectors
    [F_TNet, ~, ~, tauRes, ~] = net_track_force(terrainHypotheses(:,k), nConstantMT865 , slipHat);
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


parameterEstimate = terrainHypotheses*posteriorProbability;
peakSlipTractionEstimate = peakTractionMatHypotheses*posteriorProbability;


% ------------------- Package Recrusive Bayes Structure -------------------
structRBE.terrainProbability(:,timeStepNo) = posteriorProbability;
structRBE.parameterEstimate(:,timeStepNo) = parameterEstimate.';
structRBE.peakSlipTractionEstimate(:,timeStepNo) = peakSlipTractionEstimate;
structRBE.liklihood(:,timeStepNo) = liklihood;
structRBE.normalization(timeStepNo) = normalization;

end

