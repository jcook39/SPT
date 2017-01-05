function [ A ,x, b, LHSP, RHSP] = EOMDerivTractorTrailerMultiBody1Frame1DimBFWinch()
% Derive Equations of Motion using Lagranges Method

% Global Frame - I, J, K

% Declare Generalized Coordinates and Derivatives
    % q1 - Tractor position in 1D
        syms q1 dq1 d2q1 
    % q2 - Winch Angular Position
        syms q2 dq2 d2q2
    % q3 - Sled Position in 1D
        syms q3 dq3 d2q3
    % (Vector of Generalized Coordinates)
        q = [q1 q2 q3];
        dq = [dq1 dq2 dq3];
    

% Declare Parameters for Tractor
syms mT bTT lTT 

%Declare Parameters for Winch
syms TCG2Winch Ten Iw rw

% Declare parameters for Sled
syms mS  bSled lSled 

%Declare Global Unit Vectors
I = [1 0 0].';

% Position and Velocity Vector of Tractor
rXT= [q1 0 0].'; % Global Coordinates
vXT = fulldiff(rXT,{q(1) q(2) q(3)}); % Global Coordinates

%Position and Velocity Vector of Sled
rXS = [q3 0 0].';
vXS = fulldiff(rXS,{q(1) q(2) q(3)}); % Global Coordinates

% Kinetic Energy of the System
KE = 1/2*mT*(vXT.'*vXT) + 1/2*[0 dq2 0]*[0 Iw*dq2 0].' + 1/2*mS*(vXS.'*vXS);

% q1 = XT
LHSq1 = fulldiff(diff(KE,dq(1)),{q(1) q(2) q(3)}) - diff(KE,q(1));

% q2 = Winch Angular Position
LHSq2 = fulldiff(diff(KE,dq(2)),{q(1) q(2) q(3)}) - diff(KE,q(2));

% q3 = XS
LHSq3 = fulldiff(diff(KE,dq(3)),{q(1) q(2) q(3)}) - diff(KE,q(3));

% VectorRise LHS
LHSq = [LHSq1 LHSq2 LHSq3].';


% Addtional Position Vectors in Global Coordinates to Forces
rWinch = rXT - TCG2Winch*I;

    
% Virtual Work for Each Force (Forces and the vector r to where they're
% applied in global coordinates)************************simplify(***************
    %Left Track Forces
    syms Rbl Rcl Fl
    nForce.Rbl = -Rbl*I;          nVec.Rbl = rXT;
    nForce.Rcl = -Rcl*I;          nVec.Rcl = rXT;
    nForce.Fl = Fl*I;             nVec.Fl = rXT;

    %Right Track Forces (ex: Rlrf = Resistance lateral right front)
    syms Rbr Rcr Fr
    nForce.Rbr = -Rbr*I;          nVec.Rbr = rXT;
    nForce.Rcr = -Rcr*I;          nVec.Rcr = rXT;
    nForce.Fr = Fr*I;             nVec.Fr = rXT;

    % Force from Sled
    syms Rs % Sled Forces in Sled frame 
    nForce.Sled = -Rs*I;            nVec.Sled = rXS;
    
    % Torque pulling in or breaking to let winch out
    syms Twinch 
    Qq2 = -Twinch - sign(dq2); % Change for NonNegative Option in ODE45

    genForce = zeros(1,3);
    field = fieldnames(nForce);
    for m = 1:numel(field)
       genForce = genForce + generalized_forces(nVec.(field{m}),nForce.(field{m}));
    end
        
    RHSq = [genForce(1) genForce(2)+Qq2 genForce(3)].';
    
%Velocity Constraint
syms lambda
velCon = dq1 - dq2*rw - dq3; % Changed sogm fpr dq2 for NonNegative Option ODE45
accelCon = fulldiff(velCon,{dq(1) dq(2) dq(3)});
a11 = diff(velCon,dq1);
a12 = diff(velCon,dq2);
a13 = diff(velCon,dq3);
a = [a11 a12 a13];

%RHSq = RHSq + [a11 a12 a13].'*lambda;
LHSP = LHSq;
RHSP = (RHSq + a.'*lambda);

EqSolve = LHSq - RHSq;
    
    [M, F] = equationsToMatrix(EqSolve(1) == 0, EqSolve(2) == 0,EqSolve(3) == 0,  ...
    [d2q1 d2q2 d2q3]);
    
    A = [M -a.';-a zeros(1,1)];
    b = [F; 0];
    x = [d2q1 d2q2 d2q3 lambda].';

end

    
function [genForce] = generalized_forces(r,Fvec)
% This function computes one dot(Fvec, delr) where
% delr = sum_j[d/dq(r)]delq 

syms q1 q2 q3
     Qq1 = Fvec.'*diff(r,q1);
     Qq2 = Fvec.'*diff(r,q2);
     Qq3 = Fvec.'*diff(r,q3);
     genForce = [Qq1 Qq2 Qq3];
end