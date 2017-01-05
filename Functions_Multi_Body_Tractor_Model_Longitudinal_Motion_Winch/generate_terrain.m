function[nConstantTerrain] = generate_terrain(nConstantTerrain,nConstantMT865)

terrainMode = nConstantTerrain.Mode;



if strcmp('Constant',terrainMode)
    nConstantTerrain = generate_terrain_constant(nConstantTerrain);
elseif strcmp('Region',terrainMode)
    nConstantTerrain = generate_terrain_grid(nConstantTerrain,nConstantMT865);
elseif strcmp('TerrainVary1',terrainMode)
    nConstantTerrain = generate_terrain_varying_one(nConstantTerrain);
    nConstantTerrain = terrain_mixing(nConstantTerrain);
elseif strcmp('TerrainVary2',terrainMode)
    nConstantTerrain = generate_terrain_varying_two(nConstantTerrain);
    nConstantTerrain = terrain_mixing(nConstantTerrain);
end
    


end

function[nConstantTerrain] = generate_terrain_varying_two(nConstantTerrain)

nominalConstantFricitonAngle = nConstantTerrain.nominalFrictionAngle;
correlationCoefficient = nConstantTerrain.correlationCoefficient;
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
gridResolutionM = nConstantTerrain.gridResolutionM;



%Place peaks for 2 dimensional multivariate Guassian Distrubtions
coordinatePeak(1,:) = [50 100];
coordinatePeak(2,:) = [200 180];
coordinatePeak(3,:) = [300 25];
coordinatePeak(4,:) = [425 140];
coordinatePeak(5,:) = [250 110];

%Number of peaks placed
nPeak = length(coordinatePeak);

%Pick standard deviation of spread of peaks [Xsigma, YSigma]
peakSigma(1,1:2) = [20 30];
peakSigma(2,1:2) = [70 20];
peakSigma(3,1:2) = [60 25];
peakSigma(4,1:2) = [20 30];
peakSigma(5,1:2) = [10 40];

%Choose the amount to amplify or weight each peak
peakMultiplier = [200000 -300000 -200000 200000 -5000].';

%Build 3-dimension surface
[X , Y] = meshgrid(0:gridResolutionM:gridSizeXM,0:gridResolutionM:gridSizeYM);
terrainFrictionAngle = nominalConstantFricitonAngle;
for peakNo = 1:nPeak
    terrainFrictionAngle = terrainFrictionAngle + peakMultiplier(peakNo)*gaussian_distribution(X,coordinatePeak(peakNo,1),peakSigma(peakNo,1)).*gaussian_distribution(Y,coordinatePeak(peakNo,2),peakSigma(peakNo,2));
end
terrainCohesion = correlationCoefficient.*terrainFrictionAngle;

nConstantTerrain.frictionAngle = terrainFrictionAngle;
nConstantTerrain.cohesion = terrainCohesion;
nConstantTerrain.X = X;
nConstantTerrain.Y = Y;

end

function[nConstantTerrain] = generate_terrain_varying_one(nConstantTerrain)

nominalConstantFricitonAngle = nConstantTerrain.nominalFrictionAngle;
correlationCoefficient = nConstantTerrain.correlationCoefficient;
gridSizeXM = nConstantTerrain.grideSizeXM;
gridSizeYM = nConstantTerrain.grideSizeYM;
gridResolutionM = nConstantTerrain.gridResolutionM;


%Place peaks for 2 dimensional multivariate Guassian Distrubtions
coordinatePeak(1,:) = [100 100];
coordinatePeak(2,:) = [350 100];

%Number of peaks placed
nPeak = length(coordinatePeak);

%Pick standard deviation of spread of peaks [Xsigma, YSigma]
peakSigma(1,1:2) = [50 100];
peakSigma(2,1:2) = [75 100];

%Choose the amount to amplify or weight each peak
peakMultiplier = [60000 -300000].';

%Build 3-dimension surface
[X , Y] = meshgrid(0:gridResolutionM:gridSizeXM,0:gridResolutionM:gridSizeYM);
terrainFrictionAngle = nominalConstantFricitonAngle;
for peakNo = 1:nPeak
    terrainFrictionAngle = terrainFrictionAngle + peakMultiplier(peakNo)*gaussian_distribution(X,coordinatePeak(peakNo,1),peakSigma(peakNo,1)).*gaussian_distribution(Y,coordinatePeak(peakNo,2),peakSigma(peakNo,2));
end
terrainCohesion = correlationCoefficient.*terrainFrictionAngle;

nConstantTerrain.frictionAngle = terrainFrictionAngle;
nConstantTerrain.cohesion = terrainCohesion;
nConstantTerrain.X = X;
nConstantTerrain.Y = Y;

end

function nConstantTerrain = generate_terrain_constant(nConstantTerrain)

nominalConstantFricitonAngle = nConstantTerrain.nominalFrictionAngle;
correlationCoefficient = nConstantTerrain.correlationCoefficient;
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
gridResolutionM = nConstantTerrain.gridResolutionM;

[X , Y] = meshgrid(0:gridResolutionM:gridSizeXM,0:gridResolutionM:gridSizeYM);
magHold = nominalConstantFricitonAngle./X;
terrainFrictionAngle = X.*magHold;
terrainCohesion = terrainFrictionAngle*correlationCoefficient;

nConstantTerrain.frictionAngle = terrainFrictionAngle;
nConstantTerrain.cohesion = terrainCohesion;
nConstantTerrain.X = X;
nConstantTerrain.Y = Y;

end

function nConstantTerrain = generate_terrain_grid(nConstantTerrain,nConstantMT865)
    
    % Unpack Needed Constant Tractor Parameters
    trackAreaM2 = nConstantMT865.trackAreaM2;
    trackLengthM = nConstantMT865.trackLengthM;
    normalForceTrackN = nConstantMT865.normalForceTrackN;
    RsledN = nConstantMT865.RsledN;
    
    % Unpack Terrain Creation Parameters
    B2 = nConstantTerrain.posXBoundry(2);
    B3 = nConstantTerrain.posXBoundry(3);
    frictionAngle1 = nConstantTerrain.frictionAngle(1);
    frictionAngle2 = nConstantTerrain.frictionAngle(2);
    frictionAngle3 = nConstantTerrain.frictionAngle(3);
    cohesion1 = nConstantTerrain.cohesion(1);
    cohesion2 = nConstantTerrain.cohesion(2);
    cohesion3 = nConstantTerrain.cohesion(3);
    K1 = nConstantTerrain.K(1);
    K2 = nConstantTerrain.K(2);
    K3 = nConstantTerrain.K(3);
    keq1 = nConstantTerrain.keq(1);
    keq2 = nConstantTerrain.keq(2);
    keq3 = nConstantTerrain.keq(3);
    n1 = nConstantTerrain.n(1);
    n2 = nConstantTerrain.n(2);
    n3 = nConstantTerrain.n(3);
    S1 = nConstantTerrain.S(1);
    S2 = nConstantTerrain.S(2);
    S3 = nConstantTerrain.S(3);
    
    gridSizeXM = nConstantTerrain.gridSizeXM;
    gridSizeYM = nConstantTerrain.gridSizeYM;
    gridResolutionM = nConstantTerrain.gridResolutionM;
    
   
    % Lay Out Grid
    [X , Y] = meshgrid(0:gridResolutionM:gridSizeXM,0:gridResolutionM:gridSizeYM); 
   
    % Friction Angle
    terrainFrictionAngle = zeros(size(X));
    terrainFrictionAngle(X < B2) = frictionAngle1;
    terrainFrictionAngle((X >= B2) & (X < B3)) = frictionAngle2;
    terrainFrictionAngle(X >= B3) = frictionAngle3;
    nConstantTerrain.terrainFrictionAngle = terrainFrictionAngle;
  
    % Cohesion
    terrainCohesion = zeros(size(X));
    terrainCohesion(X < B2) = cohesion1;
    terrainCohesion((X >= B2) & (X < B3)) = cohesion2;
    terrainCohesion(X >= B3) = cohesion3;
    nConstantTerrain.terrainCohesion = terrainCohesion;
    
    % Shear Deformation Modulus: K (cm)
    terrainK = zeros(size(X));
    terrainK(X < B2) = K1;
    terrainK((X >= B2) & (X < B3)) = K2;
    terrainK(X >= B3) = K3;
    nConstantTerrain.terrainK = terrainK;
    
    % keq (M)
    terrainkeq = zeros(size(X));
    terrainkeq(X < B2) = keq1;
    terrainkeq((X >= B2) & (X < B3)) = keq2;
    terrainkeq(X >= B3) = keq3;
    nConstantTerrain.terrainkeq = terrainkeq;
    
    % n (M)
    terrainn = zeros(size(X));
    terrainn(X < B2) = n1;
    terrainn((X >= B2) & (X < B3)) = n2;
    terrainn(X >= B3) = n3;
    nConstantTerrain.terrainn = terrainn;
    
    % S (M)
    terrainS = zeros(size(X));
    terrainS(X < B2) = S1;
    terrainS((X >= B2) & (X < B3)) = S2;
    terrainS(X >= B3) = S3;
    nConstantTerrain.terrainS = terrainS;
    
    nConstantTerrain.X = X;
    nConstantTerrain.Y = Y;

end

function [z] = gaussian_distribution( X, meanX, stdDevX)
% This function returns values for a univariate PDF
    z = (1/(stdDevX*sqrt(2*pi)))*exp(-((X-meanX).^2)/(2*stdDevX^2));
end



function [nConstantTerrain] = terrain_mixing(nConstantTerrain)

nominalConstantFricitonAngleM = nConstantTerrain.nominalFrictionAngle;
correlationCoefficient = nConstantTerrain.correlationCoefficient;
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
gridResolutionM = nConstantTerrain.gridResolutionM;
terrainCohesionM = nConstantTerrain.cohesion;
terrainFrictionAngle = nConstantTerrian.frictionAngle;

%Develop Irregular Terrain
%Over 21
stdDev = 0.4;

Indicies = find((terrainFrictionAngle > 21));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 21.5 + stdDev*randn;
end
%Between 20 and 21
Indicies = find((terrainFrictionAngle <= 21) & (terrainFrictionAngle > 20));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 20.5 + stdDev*randn;
end
%Between 19 and 20
Indicies = find((terrainFrictionAngle <= 20) & (terrainFrictionAngle > 19));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 19.5 + stdDev*randn;
end
%Between 18 and 19
Indicies = find((terrainFrictionAngle <= 19) & (terrainFrictionAngle > 18));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 18.5 + stdDev*randn;
end
%Between 17 and 18
Indicies = find((terrainFrictionAngle <= 18) & (terrainFrictionAngle > 17));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 17.5 + stdDev*randn;
end
%Between 16 and 17
Indicies = find((terrainFrictionAngle <= 17) & (terrainFrictionAngle > 16));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 16.5 + stdDev*randn;
end
%Between 15 and 16
Indicies = find((terrainFrictionAngle <= 16) & (terrainFrictionAngle > 15));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 15.5 + stdDev*randn;
end
%Between 14 and 15
Indicies = find((terrainFrictionAngle <= 15) & (terrainFrictionAngle > 14));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 14.5 + stdDev*randn;
end
%Between 13 and 14
Indicies = find((terrainFrictionAngle <= 14));
nIndex = length(Indicies);
for IndexNo = 1:nIndex
     terrainFrictionAngle(Indicies(IndexNo)) = 13.5 + stdDev*randn;
end

terrainCohesion = terrainFrictionAngle*correlationCoefficient;

nConstantTerrain.frictionAngle = terrainFrictionAngle;
nConstantTerrain.cohesion = terrainCohesion;
end

