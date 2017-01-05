function [MBF, FBF, M , F] = EOMDerivTractorTrailerMultiBody1FrameBF()
% Derive Equations of Motion using Lagranges Method
%       T - Kinetic Energy 1/2*m*vg*vg + 1/2*w*Hg
%       V - Potential Energy m*g*h (datum dependent)
%       TT - tractor
%       O - Origin of global coordinates

% Global Frame - I, J, K
% Tractor Body Fixed Frame - i1, j1, k1
% Tow Arm Body Fixed Frame - i2, j2, k2
% Sled Body Fixed Frame - i3, j3, k3 


% Declare Generalized Coordinates and Derivatives
    % (Tractor)
    syms XTT dXTT d2XTT YTT dYTT d2YTT theta dtheta d2theta
    % (Sled)
    syms phi dphi d2phi si dsi d2si
    % (Vector of Generalized Coordinates)
    q = [XTT YTT theta phi si];
    dq = [dXTT dYTT dtheta dphi dsi];
    
% Declare End State Variables
syms vx dvx vy dvy

% Declare Parameters for Tractor
syms mT JT bT LT TCG2H

% Declare parameters for Sled
syms mSled JSled bSled lSled lTool

% Declare Parameters for Arm
syms lArm 

% Declare Rotation Transformation Matrices
RO_1 = rotation_matrix_z(theta); % global to frame 1  - theta is orientation of vehicle
R1_2 = rotation_matrix_z(phi);  % frame1 to frame 2 - phi is orientation of arm relative to vehicle
R1_3 = rotation_matrix_z(si); % frame1 to frame 3 - psi is orientation of sled relative to vehicle

R1_O = RO_1.';                      % frame 1 to global
R2_O = (R1_2*RO_1).';               % frame 2 to global
R3_O = (R1_3*RO_1).';          % frame 3 to global

% Compress Notation for Basis Vectors (These are not actually equal to each
% other!)
i1 = [1 0 0].'; i2 = i1; i3 = i1; I = i1;
j1 = [0 1 0].'; j2 = j1; j3 = j1; J = j1;
k1 = [0 0 1].'; k2 = k1; k3 = k1; K = k1;

% Position and Velocity Vector of Tractor
rTT_O = [XTT YTT 0].'; % Global Coordinates
vTT = fulldiff(rTT_O,{q(1), q(2), q(3), q(4), q(5)}); % Global Coordinates

% Kinetic Energy of the Tractor
wTT_O = [0 0 dtheta].';
TTT = 1/2*mT*(vTT.'*vTT) + 1/2*(wTT_O.'*wTT_O)*JT;

% Kinetic Energy of the Sled
    % Position vector to the sled in terms of generalized coordinates
    rSled_O = rTT_O + R1_O*TCG2H*-i1 + R2_O*lArm*-i2...
        + R3_O*((lSled/2) + lTool)*-i3;
    vSled = fulldiff(rSled_O,{q(1) q(2) q(3) q(4) q(5)});
    
    % Rotation Rate of Sled
    wSledArm = [0 0 dsi].'; wArmT = [0 0 dphi].'; wTT_O; % Since Planar K = k1, k2 and k3
    %wSled_O = wSledArm + wArmT + wTT_O;
    wSled_O = [0 0 dsi].' + wTT_O;

TSled = 1/2*mSled*(vSled.'*vSled) + 1/2*(wSled_O'*wSled_O)*JSled;


T_total = TTT + TSled; % The arm is assumed to be massless relative the tractor and sled

% q1 = XTT
LHSq1 = fulldiff(diff(T_total,dq(1)),{q(1) q(2) q(3) q(4) q(5)}) - diff(T_total,q(1));

%q2 = YTT
LHSq2 = fulldiff(diff(T_total,dq(2)),{q(1) q(2) q(3) q(4) q(5)}) - diff(T_total,q(2));

%q3 = thetaTT
LHSq3 = fulldiff(diff(T_total,dq(3)),{q(1) q(2) q(3) q(4) q(5)}) - diff(T_total,q(3));

%q4 = phiArm
LHSq4 = fulldiff(diff(T_total,dq(4)),{q(1) q(2) q(3) q(4) q(5)}) - diff(T_total,q(4));

%q5 = psiSled
LHSq5 = fulldiff(diff(T_total,dq(5)),{q(1) q(2) q(3) q(4) q(5)}) - diff(T_total,q(5));

% Generalized Forces: i1,j1,k1 is the body fixed frame to Tractor
    %Position Vectors in Global Coordinates
    rTT = rTT_O;

    rTT_lefTTrackCenter = rTT + R1_O*bT/2*j1;
    rTT_righTTrackCenter = rTT - R1_O*bT/2*j1;

    rTT_lefTTrackRearMid = rTT_lefTTrackCenter - R1_O*LT/4*i1;
    rTT_righTTrackRearMid = rTT_righTTrackCenter - R1_O*LT/4*i1;

    rTT_lefTTrackFrontMid = rTT_lefTTrackCenter + R1_O*LT/4*i1;
    rTT_righTTrackFrontMid = rTT_righTTrackCenter + R1_O*LT/4*i1;

    rTT_lefTTrackFront = rTT_lefTTrackCenter + R1_O*LT/2*i1;
    rTT_righTTrackFront = rTT_righTTrackCenter + R1_O*LT/2*i1;

    rTTSled = rSled_O;
    
    % Virtual Work for Each Force (Forces and the vector r to where they're
    % applied in global coordinates)***************************************
        %Left Track Forces
        syms Rbl Rcl RL RlLF RlLR Fl
        % nForce.Rbl = R1_O*-Rbl*i1;          nVec.Rbl = rTT_lefTTrackFront;
        % nForce.Rcl = R1_O*-Rcl*i1;     nVec.Rcl = rTT_lefTTrackCenter;
        nForce.RL = R1_O*-RL*i1;      nVec.RL = rTT_lefTTrackFront;
        nForce.RlLF = R1_O*RlLF*j1;   nVec.RlLF = rTT_lefTTrackFrontMid;
        nForce.RlLR = R1_O*RlLR*j1;         nVec.RlLR = rTT_lefTTrackRearMid;
        nForce.Fl = R1_O*Fl*i1;              nVec.Fl = rTT_lefTTrackCenter;
        
        %Right Track Forces (ex: Rlrf = Resistance lateral right front)
        syms Rbr Rcr RR RlRF RlRR Fr
        % nForce.Rbr = R1_O*-Rbr*i1;          nVec.Rbr = rTT_righTTrackFront;
        % nForce.Rcr = R1_O*-Rcr*i1;          nVec.Rcr = rTT_righTTrackCenter;
        nForce.RR = R1_O*-RR*i1;            nVec.RR = rTT_righTTrackFront;
        nForce.RlRF = R1_O*RlRF*j1;        nVec.RlRF = rTT_righTTrackFrontMid;
        nForce.RlRR = R1_O*RlRR*j1;         nVec.RlRR = rTT_righTTrackRearMid;
        nForce.Fr = R1_O*Fr*i1;             nVec.Fr = rTT_righTTrackCenter;
        
        % Force on Body
        syms RSledx3 RSledy3 % Sled Forces in Sled frame 
        nForce.Sled = R3_O*RSledx3*i3 + R3_O*RSledy3*j3;   nVec.Sled = rTTSled;
              
        genForce = zeros(1,5);
        field = fieldnames(nForce);
        for m = 1:numel(field)
           genForce = genForce + generalized_forces(nVec.(field{m}),nForce.(field{m}));
        end
        
    RHSq1 = genForce(1);
    RHSq2 = genForce(2);
    RHSq3 = genForce(3);
    RHSq4 = genForce(4);
    RHSq5 = genForce(5);
        
    % Place Differential Equations back into body fixed Axes for q1 and q2
    % (Repalce d2XTT with dvx)
    
    RHSq1BF = RO_1*RHSq1*I; LHSq1BF = RO_1*LHSq1*I; % XTT is purely an ODE in global unit vector I
    RHSq2BF = RO_1*RHSq2*J; LHSq2BF = RO_1*LHSq2*J; % YTT is purely an ODE in global unit vector J
    
    RHSiBF = RHSq1BF(1) + RHSq2BF(1); % Right hand side "i" unit vector body fixed
    LHSiBF = LHSq1BF(1) + LHSq2BF(1); % Left hand side "i" unit vector body fixed
    
    RHSjBF = RHSq1BF(2) + RHSq2BF(2); % Right hand side "j" unit vector body fixed
    LHSjBF = LHSq1BF(2) + LHSq2BF(2); % Left hand side "j" unit vector body fixed

    vTT2 = R1_O*vx*i1 + R1_O*vy*j1; %Body fixed coordinates
    Xd = vTT2(1)
    Yd = vTT2(2)
    Xdd = fulldiff(Xd,{vx,vy,theta,phi,si})
    Ydd = fulldiff(Yd,{vx,vy,theta,phi,si})
    
    Eq1mod = subs(LHSiBF - RHSiBF, [d2XTT d2YTT], [Xdd Ydd]);
    Eq2mod = subs(LHSjBF - RHSjBF, [d2XTT d2YTT], [Xdd Ydd]);
    Eq3mod = subs(LHSq3 - RHSq3, [d2XTT d2YTT], [Xdd Ydd]);
    Eq4mod = subs(LHSq4 - RHSq4, [d2XTT d2YTT], [Xdd Ydd]);
    Eq5mod = subs(LHSq5 - RHSq5, [d2XTT d2YTT], [Xdd Ydd]); 
    
    [MBF, FBF] = equationsToMatrix([ Eq1mod == 0 ,  Eq2mod == 0 , Eq3mod == 0, ...
        Eq4mod == 0 , Eq5mod == 0 ],[dvx dvy d2theta d2phi d2si]);
    
    MBF = simplify(MBF);
    FBF = simplify(FBF);

    Eq1 = LHSq1 - RHSq1;
    Eq2 = LHSq2 - RHSq2;
    Eq3 = LHSq3 - RHSq3;
    Eq4 = LHSq4 - RHSq4;
    Eq5 = LHSq5 - RHSq5;
    
    [M, F] = equationsToMatrix([ Eq1 == 0 , Eq2 == 0 , Eq3 == 0 , ...
        Eq4 == 0, Eq5 == 0 ],[d2XTT d2YTT d2theta d2phi d2si]);
    
    M = simplify(M);
    F = simplify(F);

end

function [R] = rotation_matrix_z(angle)
% This function constructs a Z rotational Transformation Matrix
 R = [cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];
end
    
function [genForce] = generalized_forces(r,Fvec)
% This function computes one dot(Fvec, delr) where
% delr = sum_j[d/dq(r)]delq 

syms XTT YTT theta phi si
     QXTT = Fvec.'*diff(r,XTT);
     QYTT = Fvec.'*diff(r,YTT);
     Qtheta = Fvec.'*diff(r,theta);
     Qphi = Fvec.'*diff(r,phi);
     Qsi = Fvec.'*diff(r,si);
     genForce = [QXTT QYTT Qtheta Qphi Qsi];
end