function plot_generated_trajectory(waypoint,trajColor,trajColorFiltered)

% Unpack needed waypoint structure paramters
nWayPoint = waypoint.nWayPoint;
splineCoeffX = waypoint.splineCoeffX;
splineCoeffY = waypoint.splineCoeffY;
splineCoeffXFiltered = waypoint.splineCoeffXFiltered;
splineCoeffYFiltered = waypoint.splineCoeffYFiltered;
timeWayPoint = waypoint.timeWayPoint;

trajMeshSpaceSeconds = 0.01;
nSpline = nWayPoint - 1;

% Initialize Arrays for Trajectories
trajMesh = [timeWayPoint(1,1):trajMeshSpaceSeconds:timeWayPoint(end,1)].';
trajSize = size(trajMesh,1);

XTraj = zeros(trajSize,1);
XDotTraj = zeros(trajSize,1);
XDDotTraj = zeros(trajSize,1);
YTraj = zeros(trajSize,1);
YDotTraj = zeros(trajSize,1);
YDDotTraj = zeros(trajSize,1);
velocity = zeros(trajSize,1);
gpsHeading = zeros(trajSize,1);
gpsHeadingRate = zeros(trajSize,1);

XTrajFiltered = zeros(trajSize,1);
XDotTrajFiltered = zeros(trajSize,1);
XDDotTrajFiltered = zeros(trajSize,1);
YTrajFiltered = zeros(trajSize,1);
YDotTrajFiltered = zeros(trajSize,1);
YDDotTrajFiltered = zeros(trajSize,1);
velocityFiltered = zeros(trajSize,1);
gpsHeadingFiltered = zeros(trajSize,1);
gpsHeadingRateFiltered = zeros(trajSize,1);


 for i = 1:nSpline
    
    % mesh of trajectory segment
    tI = timeWayPoint(i);
    tF = timeWayPoint(i+1);
    if i == nSpline
        trajTime = [tI:trajMeshSpaceSeconds:tF].';
        indicesTraj = ((i-1)*(numel(trajTime)-1) + 1):(i*(numel(trajTime)-1) +1);
    else
        trajTime = [tI:trajMeshSpaceSeconds:tF-trajMeshSpaceSeconds].';
        indicesTraj = ((i-1)*numel(trajTime) + 1):(i*numel(trajTime));
    end
%     trajTime = [tI:trajMeshSpaceSeconds:tF-trajMeshSpaceSeconds].';
%     indicesTraj = ((i-1)*numel(trajTime) + 1):(i*numel(trajTime));
    
    % X Trajectory
    [XTraj(indicesTraj,1), XDotTraj(indicesTraj,1), XDDotTraj(indicesTraj,1)] = compute_spline_mesh(splineCoeffX(:,i),trajTime);
    [XTrajFiltered(indicesTraj,1), XDotTrajFiltered(indicesTraj,1), XDDotTrajFiltered(indicesTraj,1)] = compute_spline_mesh(splineCoeffXFiltered(:,i),trajTime);
    
    % Y Trajectory
    [YTraj(indicesTraj,1), YDotTraj(indicesTraj,1), YDDotTraj(indicesTraj,1)] = compute_spline_mesh(splineCoeffY(:,i),trajTime);
    [YTrajFiltered(indicesTraj,1), YDotTrajFiltered(indicesTraj,1), YDDotTrajFiltered(indicesTraj,1)] = compute_spline_mesh(splineCoeffYFiltered(:,i),trajTime);    
    
    % velocity , gpsHeading, gpsHeadingRate
    [velocity(indicesTraj,1), gpsHeading(indicesTraj,1), gpsHeadingRate(indicesTraj,1)]...
        = compute_velocity_heading(XTraj(indicesTraj,1),XDotTraj(indicesTraj,1),XDDotTraj(indicesTraj,1),YTraj(indicesTraj,1),YDotTraj(indicesTraj,1),YDDotTraj(indicesTraj,1),splineCoeffX(:,i),splineCoeffY(:,i),trajTime);
    [velocityFiltered(indicesTraj,1), gpsHeadingFiltered(indicesTraj,1), gpsHeadingRateFiltered(indicesTraj,1)] ...
        = compute_velocity_heading(XTrajFiltered(indicesTraj,1),XDotTrajFiltered(indicesTraj,1),XDDotTrajFiltered(indicesTraj,1),YTrajFiltered(indicesTraj,1),YDotTrajFiltered(indicesTraj,1),YDDotTrajFiltered(indicesTraj,1),splineCoeffX(:,i),splineCoeffY(:,i),trajTime);
    
 end

figure(1)
plot(XTraj,YTraj,trajColor)
hold on

figure(121)
subplot(521)
    plot(trajMesh,XTraj,trajColor,trajMesh,XTrajFiltered,trajColorFiltered)
    ylabel('X trajectory')
    hold on
subplot(522)
    plot(trajMesh,YTraj,trajColor,trajMesh,YTrajFiltered,trajColorFiltered)
    ylabel('Y Trajectory')
    hold on
subplot(523)
    plot(trajMesh,gpsHeading,trajColor,trajMesh,gpsHeadingFiltered,trajColorFiltered)
    ylabel('gps heading')
    hold on
subplot(524)
    plot(trajMesh,velocity,trajColor,trajMesh,velocityFiltered,trajColorFiltered)
    ylabel('velocity')
    hold on
subplot(525)
    plot(trajMesh,XDotTraj,trajColor,trajMesh,XDotTrajFiltered,trajColorFiltered)
    ylabel('XDotTraj')
    hold on
subplot(526)
    plot(trajMesh,YDotTraj,trajColor,trajMesh,YDotTrajFiltered,trajColorFiltered)
    ylabel('YDotTraj') 
    hold on
subplot(527)
    plot(trajMesh,XDDotTraj,trajColor,trajMesh,XDDotTrajFiltered,trajColorFiltered)
    ylabel('XDDotTraj')
    hold on
subplot(528)
    plot(trajMesh,XDDotTraj,trajColor,trajMesh,XDDotTrajFiltered,trajColorFiltered)
    ylabel('YDDotTraj') 
    hold on
subplot(529)
    plot(trajMesh,gpsHeadingRate,trajColor,trajMesh,gpsHeadingRateFiltered,trajColorFiltered)
    ylabel('yaw Rate (Not Sure this is right)')
    hold on 


end

function [velocity, gpsHeading, yawRateRadpS] = compute_velocity_heading(XTraj,XDotTraj,XDDotTraj,YTraj,YDotTraj,YDDotTraj,splineCoeffX, splineCoeffY, trajTime)

cX0 = splineCoeffX(1,1);
cX1 = splineCoeffX(2,1);
cX2 = splineCoeffX(3,1);
cX3 = splineCoeffX(4,1);

cY0 = splineCoeffY(1,1);
cY1 = splineCoeffY(2,1);
cY2 = splineCoeffY(3,1);
cY3 = splineCoeffY(4,1);

gpsHeading = atan2(YDotTraj,XDotTraj);
velocity = XDotTraj.*cos(gpsHeading) + YDotTraj.*sin(gpsHeading);
 
gpsHeadingRateTerm1 = (1./( 1 + (YDotTraj./XDotTraj).^2 ));
gpsHeadingRateTerm2 = (1./XDotTraj).*YDDotTraj + ( -YDotTraj./(XDotTraj.^2) ).*XDDotTraj;

gpsHeadingRateTerm3 = (2*cY2 + 6*cY3*trajTime)./(3*cX3*trajTime.^2 + 2*cX2*trajTime + cX1) - ...
    ((2*cX2 + 6*cX3*trajTime).*(3*cY3*trajTime.^2 + 2*cY2*trajTime + cY2))./(3*cX3*trajTime.^2 + 2*cX2*trajTime + cX1).^2;

gpsHeadingRate = gpsHeadingRateTerm1.*(gpsHeadingRateTerm2 + gpsHeadingRateTerm3);
yawRateRadpS = gpsHeadingRate;

% yawRateRadpS = 0;
end

function [z, zDot, zDDot] = compute_spline_mesh(CZ,t)
% This function computes the mesh points along a spline segment
cZ0 = CZ(1);
cZ1 = CZ(2);
cZ2 = CZ(3);
cZ3 = CZ(4);

z = cZ0 + cZ1*t + cZ2*t.^2 + cZ3*t.^3;
zDot = cZ1 + 2*cZ2*t + 3*cZ3*t.^2;
zDDot = 2*cZ2 + 6*cZ3*t;

end