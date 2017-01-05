function [MT865] = terrain_parametrs_track(MT865, nConstantTerrain)  

%Find friciton angle under each track sprocket
MT865.terrainLeftFront = terrain_parameters(MT865.posLeftFront, nConstantTerrain);
MT865.terrainRightFront = terrain_parameters(MT865.posRightFront, nConstantTerrain);
MT865.terrainLeftRear = terrain_parameters(MT865.posLeftRear, nConstantTerrain);
MT865.terrainRightRear = terrain_parameters(MT865.posRightRear, nConstantTerrain);


end

function [ terrainParameters ] = terrain_parameters( r, nConstantTerrain)

terrainFrictionAngle = nConstantTerrain.terrainFrictionAngle;
terrainCohesion = nConstantTerrain.terrainCohesion;
terrainK = nConstantTerrain.terrainK;
terrainkeq = nConstantTerrain.terrainkeq;
terrainn = nConstantTerrain.terrainn;
terrainS = nConstantTerrain.terrainS;
gridResolutionM = nConstantTerrain.gridResolutionM;

xGlobalLocation = r(1);
yGlobalLocation = r(2);

gridLocationX = (xGlobalLocation/gridResolutionM) + 1;
indexLowerX = floor(gridLocationX);
indexUpperX = indexLowerX +1;
gridLocationY = (yGlobalLocation/gridResolutionM) + 1;
indexLowerY = floor(gridLocationY);
indexUpperY = indexLowerY + 1;

[X,Y] = meshgrid(indexLowerX:indexUpperX,indexLowerY:indexUpperY);
vFrictionAngle = [terrainFrictionAngle(Y(1,1),X(1,1)) terrainFrictionAngle(Y(1,2),X(1,2)); terrainFrictionAngle(Y(2,1),X(2,1)) terrainFrictionAngle(Y(2,2),X(2,2))];
vCohesion = [terrainCohesion(Y(1,1),X(1,1)) terrainCohesion(Y(1,2),X(1,2)); terrainCohesion(Y(2,1),X(2,1)) terrainCohesion(Y(2,2),X(2,2))];
vK = [terrainK(Y(1,1),X(1,1)) terrainK(Y(1,2),X(1,2)); terrainK(Y(2,1),X(2,1)) terrainK(Y(2,2),X(2,2))];
vkeq = [terrainkeq(Y(1,1),X(1,1)) terrainkeq(Y(1,2),X(1,2)); terrainkeq(Y(2,1),X(2,1)) terrainkeq(Y(2,2),X(2,2))];
vn = [terrainn(Y(1,1),X(1,1)) terrainn(Y(1,2),X(1,2)); terrainn(Y(2,1),X(2,1)) terrainn(Y(2,2),X(2,2))];
vS = [terrainS(Y(1,1),X(1,1)) terrainS(Y(1,2),X(1,2)); terrainS(Y(2,1),X(2,1)) terrainS(Y(2,2),X(2,2))];

frictionAngle = interp2(X,Y,vFrictionAngle,gridLocationX, gridLocationY);
cohesion = interp2(X,Y,vCohesion,gridLocationX, gridLocationY);
K = interp2(X,Y,vK,gridLocationX,gridLocationY);
keq = interp2(X,Y,vkeq,gridLocationX,gridLocationY);
n = interp2(X,Y,vn,gridLocationX,gridLocationY);
S = interp2(X,Y,vS,gridLocationX,gridLocationY);

terrainParameters = [cohesion, frictionAngle, K, keq, n, S].'; 

end
