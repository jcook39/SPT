function plot_tractor_on_map_longitudinal(tractor,i,nConstantMT865,tractorColor)
    
    % Unpack needed information from tractor Structure
    X = tractor(i).state(1);
    Y = tractor(i).state(2);
    theta = 0;
    phi = 0;
    si = 0;
    
    % Unpack Constant Tractor Paramters
    b = nConstantMT865.trackCG2trackCG;
    l = nConstantMT865.trackLengthM;
    w = nConstantMT865.trackWidthM;
    TCG2H = nConstantMT865.TTCG2Hitch;
    lSled = nConstantMT865.sledLengthM;
    bSled = nConstantMT865.sledWidthM;
    lArm = nConstantMT865.towArmM;
    lTool = nConstantMT865.CRLTool;

    % Rotational Transformation Matrices  
    R1_0 = rotation_matrix_z(theta); % Inertial frame to tractor frame
    R2_1 = rotation_matrix_z(phi);   % Tractor frame to tow arm frame
    R3_1 = rotation_matrix_z(si);    % Tractor Frame to sled frame
    R0_1 = transpose(R1_0);             % Tractor Frame to Inertial Frame
    R0_2 = transpose(R2_1*R1_0);        % Tow Arm frame to Inertial Frame
    R0_3 = transpose(R3_1*R1_0);        % Sled Frame to Intertial Frame

    % Four Corners Left Track Position
    pLT_FL = [X Y 0].' + R0_1*[l/2 ((b/2)+(w/2)) 0].';
    pLT_FR = [X Y 0].' + R0_1*[l/2 ((b/2)-(w/2)) 0].';
    pLT_RL = [X Y 0].' + R0_1*[-l/2 ((b/2)+(w/2)) 0].';
    pLT_RR = [X Y 0].' + R0_1*[-l/2 ((b/2)-(w/2)) 0].';
    fill([pLT_FL(1) pLT_FR(1) pLT_RR(1) pLT_RL(1) pLT_FL(1)],[pLT_FL(2) pLT_FR(2) pLT_RR(2) pLT_RL(2) pLT_FL(2)],'k');
    hold on

    % Four Corners Right Track Position
    pRT_FL = [X Y 0].' + R0_1*[l/2 (-(b/2)+(w/2)) 0].';
    pRT_FR = [X Y 0].' + R0_1*[l/2 (-(b/2)-(w/2)) 0].';
    pRT_RL = [X Y 0].' + R0_1*[-l/2 (-(b/2)+(w/2)) 0].';
    pRT_RR = [X Y 0].' + R0_1*[-l/2 (-(b/2)-(w/2)) 0].'; 
    fill([pRT_FL(1) pRT_FR(1) pRT_RR(1) pRT_RL(1) pRT_FL(1)],[pRT_FL(2) pRT_FR(2) pRT_RR(2) pRT_RL(2) pRT_FL(2)],'k');
    hold on

    % Plot yellow cab and engine positions
    pBD_FL = [X Y 0].' + R0_1*[2.5 0.8*w 0].';
    pBD_FR = [X Y 0].' + R0_1*[2.5 -0.8*w 0].';
    pBD_MR = [X Y 0].' + R0_1*[0.25 -0.8*w 0].';
    pCAB_FR = [X Y 0].' + R0_1*[0.25 (-b/2) 0].';
    pCAB_RR = [X Y 0].' + R0_1*[-1.25 (-b/2) 0].';
    pCAB_RL = [X Y 0].' + R0_1*[-1.25 (b/2) 0].';
    pCAB_FL = [X Y 0].' + R0_1*[0.25 (b/2) 0].';
    pBD_ML = [X Y 0].' + R0_1*[0.25 0.8*w 0].';
    fill([pBD_FL(1) pBD_FR(1) pBD_MR(1) pCAB_FR(1) pCAB_RR(1) pCAB_RL(1) pCAB_FL(1) pBD_ML(1)],...
    [pBD_FL(2) pBD_FR(2) pBD_MR(2) pCAB_FR(2) pCAB_RR(2) pCAB_RL(2) pCAB_FL(2) pBD_ML(2)],tractorColor);
    hold on

%     % Plot Tow arm and Sled
%     pA = [X Y 0].' + R0_1*[-TCG2H 0 0].'; % Pt where tow arm attaches to tractor
%     pB = pA + R0_2*[-lArm 0 0].'; % pt where tow arm attaches to sled
%     pC = pB + R0_3*[-((lSled/2)+lTool) 0 0].'; % CG of Sled
% 
%     % Truss Points
%     sledFL = pC + R0_3*[lSled/2 bSled/2 0].';
%     sledFLmid = pC + R0_3*[lSled/2 bSled/4 0].';
%     sledFR = pC + R0_3*[lSled/2 -bSled/2 0].';
%     sledFRmid = pC + R0_3*[lSled/2 -bSled/4 0].';
%     sledFC = pC + R0_3*[lSled/2 0 0].';
%     sledRL = pC + R0_3*[-lSled/2 bSled/2 0].';
%     sledRR = pC + R0_3*[-lSled/2 -bSled/2 0].';
% 
%     % Plot Trus
%     plot([pB(1) sledFL(1)],[pB(2) sledFL(2)],'r')
%     hold on
%     plot([pB(1) sledFLmid(1)],[pB(2) sledFLmid(2)],'r')
%     hold on
%     plot([pB(1) sledFR(1)],[pB(2) sledFR(2)],'r')
%     hold on
%     plot([pB(1) sledFRmid(1)],[pB(2) sledFRmid(2)],'r')
%     hold on
%     plot([pB(1) sledFC(1)],[pB(2) sledFC(2)],'r')
%     hold on
%     plot([sledFL(1) sledFR(1)],[sledFL(2) sledFR(2)],'r')
%     hold on
% 
%     % PLot Tow Arm
%     plot([pA(1) pB(1)],[pA(2) pB(2)],'k')
%     hold on
% 
%     % Sled points - (Sleds numbered going left to right)
%     space = 1.2;
%     indSledWidthM = nConstantMT865.indSledWidthM;
%     sledLengthM = nConstantMT865.sledLengthM;
% 
%     pS1_FL = sledFL;
%     pS1_FR = pS1_FL + R0_3*[0 -indSledWidthM 0].';
%     pS1_RR = pS1_FR + R0_3*[-sledLengthM 0 0].';
%     pS1_RL = sledRL;
% 
%     pS2_FL = pS1_FR + R0_3*[0 -space 0].';
%     pS2_FR = pS2_FL + R0_3*[0 -indSledWidthM 0].';
%     pS2_RR = pS2_FR + R0_3*[-sledLengthM 0 0].';
%     pS2_RL = pS2_RR + R0_3*[0 indSledWidthM 0].';
% 
%     pS4_FR = sledFR;
%     pS4_FL = pS4_FR + R0_3*[0 indSledWidthM 0].';
%     pS4_RL = pS4_FL + R0_3*[-sledLengthM 0 0].';
%     pS4_RR = sledRR;
% 
%     pS3_FR = pS4_FL + R0_3*[0 space 0].';
%     pS3_FL = pS3_FR + R0_3*[0 indSledWidthM 0].';
%     pS3_RL = pS3_FL + R0_3*[-sledLengthM 0 0].';
%     pS3_RR = pS3_RL + R0_3*[0 -indSledWidthM 0].';
% 
% 
%     %plot([sledFL(1) sledFR(1) sledRR(1) sledRL(1) sledFL(1)],[sledFL(2) sledFR(2) sledRR(2) sledRL(2) sledFL(2)],'k:')
%     %hold on
%     fill([pS1_FL(1) pS1_FR(1) pS1_RR(1) pS1_RL(1) pS1_FL(1)],[pS1_FL(2) pS1_FR(2) pS1_RR(2) pS1_RL(2) pS1_FL(2)],'k')
%     hold on
%     fill([pS2_FL(1) pS2_FR(1) pS2_RR(1) pS2_RL(1) pS2_FL(1)],[pS2_FL(2) pS2_FR(2) pS2_RR(2) pS2_RL(2) pS2_FL(2)],'k')
%     hold on
%     fill([pS3_FL(1) pS3_FR(1) pS3_RR(1) pS3_RL(1) pS3_FL(1)],[pS3_FL(2) pS3_FR(2) pS3_RR(2) pS3_RL(2) pS3_FL(2)],'k')
%     hold on
%     fill([pS4_FL(1) pS4_FR(1) pS4_RR(1) pS4_RL(1) pS4_FL(1)],[pS4_FL(2) pS4_FR(2) pS4_RR(2) pS4_RL(2) pS4_FL(2)],'k')
%     hold on
end