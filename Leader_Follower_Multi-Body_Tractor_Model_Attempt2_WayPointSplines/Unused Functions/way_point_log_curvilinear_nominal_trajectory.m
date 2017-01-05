function wayPoint = way_point_log_curvilinear_nominal_trajectory(tractorLead, controller, trajSmooth, timeStepNo)
% This function computes the waypoint for a follower vehicle based on the
% lead vehicles information

%%%%%%%%%%%%%%%%%%%%%% Unpack Controller Parameters %%%%%%%%%%%%%%%%%%%%%%%
Refp1 = controller.Refp1;
Refq1 = controller.Refq1;
Refz1 = 0;
ref = [Refp1 Refq1 Refz1].';

%%%%%%%%%%%%%%%%%%%% Unpack Leader Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%
XL = tractorLead.state(1);
YL = tractorLead.state(2);
thetaL = tractorLead.state(3);
vxL = tractorLead.state(6);
vyL = tractorLead.state(7);
dthetaL = tractorLead.state(8);

%Unpack smoothTraj Paramter
yawRateCmd = trajSmooth.yawRateCmdTraj(timeStepNo-1,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%% Compute Curvature %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speed = sqrt(vxL^2 + vyL^2);
gpsHeading = atan2(vyL,vxL) + thetaL;
% curvature = dthetaL/speed;
curvature = yawRateCmd/speed;
turningRadius = 1/curvature;

%%%%%%%%%%%%%%%%%%%% Compute WayPoint %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if abs(turningRadius) < 100000000000
    pivotPointBodyFixedFrame1 = [0 turningRadius 0].';
    pivotPointGlobalFrame = [XL YL 0].' + rotation_matrix_z(-gpsHeading)*pivotPointBodyFixedFrame1;
    
    refAngle2LeaderCurvilinearCoordinates = gpsHeading;
    refAngle2FollowerCurvilinearCoordinates = refAngle2LeaderCurvilinearCoordinates + (Refp1/turningRadius);

    XF = pivotPointGlobalFrame(1,1) + (turningRadius - Refq1)*sin(refAngle2FollowerCurvilinearCoordinates);
    YF = (pivotPointGlobalFrame(2,1) - turningRadius) + turningRadius - (turningRadius - Refq1)*cos(refAngle2FollowerCurvilinearCoordinates);

    wayPoint = [XF YF 0].';
else
    wayPoint = way_point_log_cartesian(tractorLead, controller);
end

end