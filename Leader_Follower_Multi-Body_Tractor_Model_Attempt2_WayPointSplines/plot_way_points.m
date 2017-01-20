function plot_way_points(waypoint, nWayPointFollowEval, wayPointColorLeader,wayPointColorFollower)
% Plot waypoints from the lead tractor

% Unpack waypoint structure
nWayPointStateLeader = waypoint.nWayPointStateLeader;
nWayPointStateFollower = waypoint.nWayPointStateFollower;
nWayPoint = waypoint.nWayPoint;

lineWidthnWayPoint = 2;
lineWidthSelectedWayPoint = 2;

figure(1)
    for i = 1:nWayPoint
        %plot(nWayPointStateLeader(1,i),nWayPointStateLeader(2,i),wayPointColorLeader)
        %hold on
        plot(nWayPointStateFollower(1,i),nWayPointStateFollower(2,i),wayPointColorFollower,'linewidth',lineWidthnWayPoint)
        hold on
        %plot(nWayPointFollowEval(i).wayPointFollowEval(1,1),nWayPointFollowEval(i).wayPointFollowEval(1,2),'g>')
        %hold on
    end
    plot(nWayPointFollowEval(nWayPoint-1).wayPointFollowEval(1,1),nWayPointFollowEval(nWayPoint-1).wayPointFollowEval(1,2),'bx','linewidth',lineWidthSelectedWayPoint)
    hold on
%     xArrow = [0.82 0.844];
%     yArrow = [0.3 0.44];
%     a = annotation('textarrow', xArrow, yArrow, 'String', 'selected waypoint','fontsize',16,'color','b', 'HeadLength', 20, 'HeadWidth',20, 'linewidth', 2);
%     dim = [0.835 0.442 0.015 0.015];
%     annotation('ellipse',dim,'color','b','linewidth',1)
end