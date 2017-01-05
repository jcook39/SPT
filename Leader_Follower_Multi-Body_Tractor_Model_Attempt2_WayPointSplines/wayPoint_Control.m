function [waypoint, nWayPointFollowEval] = wayPoint_Control(tractorLead, tractorFollow, waypoint, nWayPointFollowEval, controller, timeStepS, timeStepNo)


% waypoint structure unpack
wayPointTimer = waypoint.wayPointTimer;
wayPointTimeStepS = waypoint.wayPointTimeStepS;
wayPointNo = waypoint.wayPointNo;


% Gathers WayPoints at specified Intervals
if abs(wayPointTimer - wayPointTimeStepS) < (1E-8)
    wayPointNo = wayPointNo + 1;
    nWayPointStateLeader = way_point_log_leader(tractorLead);
    nWayPointStateFollower = way_point_log_follower(tractorLead,controller);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %------------------ Unpack needed tractor states-----------------------
    XFollow = tractorFollow.state(1);
    YFollow = tractorFollow.state(2);
    thetaFollow = tractorFollow.state(3);
    vxFollow = tractorFollow.state(6);
    vyFollow = tractorFollow.state(7);
    speedFollow = sqrt(vxFollow^2 + vyFollow^2);
    gpsHeadingFollow = atan2(vyFollow,vxFollow) + thetaFollow;
    
    lookAheadDistance = 40;
    if wayPointNo == 1
        nWayPointFollowEvalMat = nWayPointStateFollower(1:2,1).';
    else
        nWayPointFollowEvalMatPast = nWayPointFollowEval(wayPointNo-1).nWayPointFollowEvalMat;
        nWayPointFollowEvalMat = [nWayPointFollowEvalMatPast; nWayPointStateFollower(1:2,1).'];
    end
    
    dist2nWayPointFollowEvalMatHold(:,1) = nWayPointFollowEvalMat(:,1) - XFollow;
    dist2nWayPointFollowEvalMatHold(:,2) = nWayPointFollowEvalMat(:,2) - YFollow;
    
    angle2Points = atan2(dist2nWayPointFollowEvalMatHold(:,2) , dist2nWayPointFollowEvalMatHold(:,1) ) - gpsHeadingFollow;
    nWayPointFollowEvalMat = nWayPointFollowEvalMat(abs(angle2Points) <= pi/2,:);
    
    dist2WayPointFollowEval = sqrt( (nWayPointFollowEvalMat(:,1) - XFollow).^2 + (nWayPointFollowEvalMat(:,1) - XFollow).^2 );
    
    pointIndexWayPointFollowEval = min(abs(dist2WayPointFollowEval - lookAheadDistance)) == abs(dist2WayPointFollowEval - lookAheadDistance);
    wayPointFollowEval = nWayPointFollowEvalMat(pointIndexWayPointFollowEval,:);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %------------------- Reset Timer for Waypoint -------------------------
    wayPointTimer = 0;   
     
    %------------------- Pack waypoint structure --------------------------
    waypoint.wayPointTimer = wayPointTimer;
    waypoint.wayPointNo = wayPointNo;
    waypoint.nWayPointStateLeader(:,wayPointNo) = nWayPointStateLeader;
    waypoint.nWayPointStateFollower(:,wayPointNo) = nWayPointStateFollower;

    %------------- Pack nWayPointFollowEval structure ---------------------
    nWayPointFollowEval(wayPointNo).nWayPointFollowEvalMat = nWayPointFollowEvalMat;
    nWayPointFollowEval(wayPointNo).wayPointFollowEval = wayPointFollowEval;
    
end

% Increment waypoint timer
wayPointTimer = wayPointTimer + timeStepS;
waypoint.wayPointTimer = wayPointTimer;


end

function wayPointStateFollower = way_point_log_follower(tractorLead,controller)

% Unpack controller paramters 
Refq1 = controller.Refq1; % flexible y

%
wayPointStateLeader = way_point_log_leader(tractorLead);
XLead = wayPointStateLeader(1,1);
YLead = wayPointStateLeader(2,1);
gpsHeadingLead = wayPointStateLeader(3,1);

wayPointFollower = [XLead YLead 0].' + rotation_matrix_z(-gpsHeadingLead)*[0 Refq1 0].';
wayPointStateFollower = [wayPointFollower(1,1) wayPointFollower(2,1) 0 0 0].';

end