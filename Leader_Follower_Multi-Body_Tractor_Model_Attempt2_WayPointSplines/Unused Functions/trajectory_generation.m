function [splineCoeffX, splineCoeffY] = trajectory_generation(nWayPoint,tractorStructNewWayPoint,tractorStructOldWayPoint,timeWayPoint,wayPointNo)

% Unpack Waypoints
wayPoint1 = nWayPoint(:,1);
X_tI = wayPoint1(1,1);
Y_tI = wayPoint1(2,1);

wayPoint2 = nWayPoint(:,2);
X_tF = wayPoint2(1,1);
Y_tF = wayPoint2(2,1);

% Get Derivatives at X, Y coordinate
[XDot_tF, YDot_tF] = derivative_nPoint(tractorStructNewWayPoint);
[XDot_tI, YDot_tI] = derivative_nPoint(tractorStructOldWayPoint);

splineCoeffX = spline_fit( X_tI, X_tF, XDot_tI, XDot_tF, timeWayPoint, wayPointNo);
splineCoeffY = spline_fit( Y_tI, Y_tF, YDot_tI, YDot_tF, timeWayPoint, wayPointNo);

end

function [XDot, YDot] = derivative_nPoint(MT865)

theta = MT865.state(3);
vx = MT865.state(6);
vy = MT865.state(7);

v = sqrt( vx^2 + vy^2);
gpsHeading = atan2(vy,vx) + theta;

XDot = v*cos(gpsHeading);
YDot = v*sin(gpsHeading);

% XDot = vx*cos(theta) - vy*sin(theta);
% YDot = vx*sin(theta) + vy*cos(theta);

end

function splineCoeff = spline_fit(z_tI,z_tF,zDot_tI, zDot_tF,timeWayPoint,wayPointNo)

tI = timeWayPoint(wayPointNo-1,1);
tF = timeWayPoint(wayPointNo,1);
 
% A = [1  tI  tI^2  tI^3;
%      1  tF  tF^2  tF^3;
%      0  1   2*tI  3*tI^2; 
%      0  1   2*tF  3*tF^2];
%  
% b = [z_tI z_tF zDot_tI zDot_tF].';

A = [1  tI  tI^2  tI^3;
     1  tF  tF^2  tF^3;
     0  1   2*tI  3*tI^2; 
     0  1   2*tF  3*tF^2;
     0  0   2     6*tI;
     0  0   2     6*tF];
 
b = [z_tI z_tF zDot_tI zDot_tF  0  0].';

splineCoeff = A\b;
end