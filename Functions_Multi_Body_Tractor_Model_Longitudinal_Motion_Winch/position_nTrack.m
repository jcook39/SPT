function [ MT865 ] = position_nTrack( MT865, nConstantMT865 )
% This function calculates the position of each segment of the track. Each
% track is divided into two segements: the front and the rear


trackLengthM = nConstantMT865.trackLengthM;
b = nConstantMT865.trackCG2trackCG;
X = MT865.state(1);
Y = MT865.state(2);
theta = MT865.state(3);


% Global Coordinate of CG of tractor
I = [1 0 0].';
J = [0 1 0].';
rT = X*I + Y*J;

% Vectors to track Segments in BF axis
i = [1 0 0].'; 
j = [0 1 0].'; 

rLeftFront = (trackLengthM/4)*i + (b/2)*j;
rRightFront = (trackLengthM/4)*i - (b/2)*j;
rLeftRear = -(trackLengthM/4)*i + (b/2)*j;
rRightRear = -(trackLengthM/4)*i - (b/2)*j;

% Rotational Transmformation matrix from Global to frame 1
Rz = [cos(theta) sin(theta) 0;...
     -sin(theta) cos(theta) 0;...
     0              0       1;];

% Global Position Vectors for 4 track segments
MT865.posLeftFront = rT + Rz.'*rLeftFront;
MT865.posRightFront = rT + Rz.'*rRightFront;
MT865.posLeftRear = rT + Rz.'*rLeftRear;
MT865.posRightRear = rT + Rz.'*rRightRear;

end

