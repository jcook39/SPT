function wayPoint = way_point_log_cartesian(tractorLead, controller)
% This function computes the waypoint for a follower vehicle based on the
% lead vehicles information

%%%%%%%%%%%%%%%%%%%%%% Unpack Controller Parameters %%%%%%%%%%%%%%%%%%%%%%%
Refx1 = controller.Refx1;
Refy1 = controller.Refy1;
Refz1 = 0;
ref = [Refx1 Refy1 Refz1].';

%%%%%%%%%%%%%%%%%%%% Unpack Leader Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%
XL = tractorLead.state(1);
YL = tractorLead.state(2);
thetaL = tractorLead.state(3);
vxL = tractorLead.state(6);
vyL = tractorLead.state(7);

gpsHeading = atan2(vyL,vxL) + thetaL;


%%%%%%%%%%%%%%%%%%%% Compute WayPoint %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
offsetGlobalFrame = rotation_matrix_z( -gpsHeading )*ref;
wayPoint = [XL YL 0].' +  offsetGlobalFrame;

end