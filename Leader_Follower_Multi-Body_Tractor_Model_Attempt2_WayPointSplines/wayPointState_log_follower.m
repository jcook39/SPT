function wayPointStateFollower = wayPointState_log_follower(waypoint,controller)

% Unpack offset controller parameters
Refp1 = controller.Refp1; % flexible x
Refq1 = controller.Refq1; % flexible y

% Unpack waypoint paramters
wayPointNo = waypoint.wayPointNo;
nWayPointStateLeader = waypoint.nWayPointStateLeader;
XLead = nWayPointStateLeader(1,1:wayPointNo).';
YLead = nWayPointStateLeader(2,1:wayPointNo).';
headingLead = nWayPointStateLeader(3,1:wayPointNo).';

XLeadFlip = flipud(XLead);
YLeadFlip = flipud(YLead);
headingLeadFlip = flipud(headingLead);

% Distance between waypoints
cumDist = 0;
distance = zeros(wayPointNo-1,1);
if wayPointNo < 2
    wayPointStateFollower = zeros(5,1)*NaN;
else
    for j = 1:wayPointNo-1
       distance(j,1) = sqrt( ( XLeadFlip(j+1,1) - XLeadFlip(j,1) )^2 + ( YLeadFlip(j+1,1) - YLeadFlip(j,1) )^2 );
       cumDist = distance(j,1) + cumDist;
       if cumDist > abs(Refp1)
           % Inerpolate between point j+1 and j
           distInterp = abs(Refp1) - sum(distance(1:j-1,1));
           XHat = ( XLeadFlip(j+1,1) - XLeadFlip(j,1) )/distance(j,1);
           YHat = ( YLeadFlip(j+1,1) - YLeadFlip(j,1) )/distance(j,1);
           headingHat = ( headingLeadFlip(j+1,1) - headingLeadFlip(j,1) )/distance(j,1);
           XFollowPoint_p1 = distInterp*XHat + XLeadFlip(j,1) ;
           YFollowPoint_p1 = distInterp*YHat + YLeadFlip(j,1) ;
           headingFollowPoint_p1 = distInterp*headingHat + headingLeadFlip(j,1);        
           followPoint_p1q1 = [XFollowPoint_p1 YFollowPoint_p1 0].' + rotation_matrix_z(-headingFollowPoint_p1)*[0 Refq1 0].';
           
           speed = 0;
           yawRate = 0;
           
           wayPointStateFollower = [followPoint_p1q1; speed; yawRate];
           break
       else
           wayPointStateFollower = zeros(5,1)*NaN;
       end
    end
end



end