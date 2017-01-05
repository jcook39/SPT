function wayPointState = way_point_log_leader(tractor)

% Unpack tractor State Information
X = tractor.state(1);
Y = tractor.state(2);
theta = tractor.state(3);
vx = tractor.state(6);
vy = tractor.state(7);
thetaDot = tractor.state(8);

% Compute measureable values of speed and heading angle
gpsHeading = atan2(vy,vx) + theta;
speed = sqrt(vx^2 + vy^2);

wayPointState = [X Y gpsHeading speed thetaDot].';

end