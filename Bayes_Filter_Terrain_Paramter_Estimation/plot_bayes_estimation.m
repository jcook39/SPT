function plot_bayes_estimation(tractor,structRBE,nConstantMT865,nTimeParam,estimateColor,trueColor,histString,figureNo)

% =================== Unpacking and Declaration ===========================

% ----------------------- Unpack Time Parameters --------------------------
nTimeStep = nTimeParam.nTimeStep;
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;


% -------------------------- Estimates ------------------------------------
cohesionEstimate = structRBE.parameterEstimate(1,:);
frictionAngleEstimate = structRBE.parameterEstimate(2,:);
nEstimate = structRBE.parameterEstimate(3,:);
keqEstimate = structRBE.parameterEstimate(4,:);
KEstimate = structRBE.parameterEstimate(5,:);
SEstimate = structRBE.parameterEstimate(6,:);

% This is the slip value at which the vehicle operates with maximum
% traction force with no load (results show no difference with load)
peakSlip = structRBE.peakSlip;
netTractionNoLoadEstimateMax = structRBE.netTractionNoLoadEstimateMax;
slipVectorBayes = structRBE.slipVectorBayes;

% ------------------ Terrain Parameters True  -----------------------------
indexVector = 1:numel(time);
nTimeStep = numel(time);
for i = 1:nTimeStep
    terrainCohesion(i) = tractor(i).terrainLeftFront(1);
    terrainFrictionAngle(i) = tractor(i).terrainLeftFront(2);
    terrainK(i) = tractor(i).terrainLeftFront(3);
    terrainkeq(i) = tractor(i).terrainLeftFront(4);
    terrainn(i) = tractor(i).terrainLeftFront(5);
    terrainS(i) = tractor(i).terrainLeftFront(6);
    terrainVector(:,i) = [terrainCohesion(i) terrainFrictionAngle(i) terrainn(i) terrainkeq(i) terrainK(i) terrainS(i)].';
    [netTractionNoLoadTrueMax, peakSlipNoLoadTrue, netTractionLoadTrue, peakSlipLoadTrue] = peak_traction(nConstantMT865, terrainVector(:,i), slipVectorBayes, 'MaxTraction');
    peakTractionMatTrueMax(:,i) = [netTractionNoLoadTrueMax  peakSlipNoLoadTrue  netTractionLoadTrue  peakSlipLoadTrue].';
end

netTractionNoLoadTrueMax = peakTractionMatTrueMax(1,:);
peakSlipNoLoadTrue = peakTractionMatTrueMax(2,:);
netTractionLoadTrue = peakTractionMatTrueMax(3,:); 
peakSlipLoadTrue = peakTractionMatTrueMax(4,:);

% ---------------------------- Terrain Hypotheses -------------------------
terrainHypotheses = structRBE.terrainHypotheses;
nHypotheses = structRBE.nHypotheses;
c = terrainHypotheses(1,:);
phi = terrainHypotheses(2,:);
n = terrainHypotheses(3,:);
keq = terrainHypotheses(4,:);
K = terrainHypotheses(5,:);
S = terrainHypotheses(6,:);

netTractionNoLoadHypotheses = structRBE.peakTractionMatHypotheses(1,:);
peakSlipNoLoadHypotheses = structRBE.peakTractionMatHypotheses(2,:);
netTractionLoadHypotheses = structRBE.peakTractionMatHypotheses(3,:);
peakSlipLoadHypotheses = structRBE.peakTractionMatHypotheses(4,:);

% ----------------------- Plot Commands -----------------------------------

lineWidthSize = 4;
fontLabel = 16;

figure(figureNo)
subplot(231)
    h = plot(time,cohesionEstimate, estimateColor, time, terrainCohesion(indexVector), trueColor);
    set(h(2),'linewidth',lineWidthSize)
    legend('Estimate','Actual','location','southwest')
    ylim([0 8])
    ylabel('cohesion','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    hold on
subplot(232)
    h = plot(time,frictionAngleEstimate, estimateColor, time, terrainFrictionAngle(indexVector), trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylabel('friction Angle','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    ylim([10 25])
    hold on
subplot(233)
    h = plot(time,nEstimate, estimateColor, time, terrainn(indexVector), trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylabel('n','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    ylim([0.5 1.5])
    hold on
subplot(234)
    h = plot(time,keqEstimate, estimateColor, time, terrainkeq(indexVector), trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylabel('keq ','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    ylim([100 600])
    hold on
subplot(235)
    h = plot(time,KEstimate, estimateColor, time, terrainK, trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylabel('K ','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    hold on
subplot(236)
    h = plot(time,SEstimate, estimateColor, time, terrainS(indexVector), trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylabel('S','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    hold on
    
    
% if strcmp(histString,'plotHist')
% figure(figureNo+1)
% subplot(121)
%     histogram(peakSlipNoLoadHypotheses,[0:40])
%     ylabel('Count No')
%     xlabel('Peak Traction Slip Value No Payload')
% subplot(122)
%     histogram(peakSlipLoadHypotheses,[0:40])
%     ylabel('Count No')
%     xlabel('Peak Traction Slip Value with Payload')
% end

if strcmp(histString,'plotHist')
figure(figureNo+1)
    histogram(peakSlipNoLoadHypotheses,[0:45])
    ylabel('Count No')
    xlabel('Peak Traction Slip')
end
    

figure(figureNo+2)
subplot(121)
    h = plot(time, peakSlip, estimateColor, time, peakSlipLoadTrue, trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylim([0 30])
    legend('Estimate','True')
    ylabel('Peak Slip (\%)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    hold on
subplot(122)
    h = plot(time, netTractionNoLoadEstimateMax, estimateColor, time, netTractionNoLoadTrueMax, trueColor);
    set(h(2),'linewidth',lineWidthSize)
    ylabel('Peak Net Traction (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    hold on
    
% endValue = time(end);
% timeAugmented = 0:2:endValue;
% HL = ones(numel(timeAugmented),1);
% figure(figureNo)
% subplot(231)
%     plot(timeAugmented, c(1)*HL, '*', timeAugmented, c(2)*HL, '*', timeAugmented, c(3)*HL, '*', timeAugmented, c(4)*HL, '*', timeAugmented, c(5)*HL, '*',...
%         time,cohesionEstimate, estimateColor, time, terrainCohesion(indexVector), trueColor)
%     legend('Hyp1','Hyp2','Hyp3','Hyp4','Hyp5','Estimate','Actual','location','southwest')
%     ylim([0 8])
%     ylabel('cohesion (kPa)')
%     xlabel('time (seconds)')
% subplot(232)
%     plot(timeAugmented, phi(1)*HL, '*', timeAugmented, phi(2)*HL, '*', timeAugmented, phi(3)*HL, '*', timeAugmented, phi(4)*HL, '*', timeAugmented, phi(5)*HL, '*',...
%         time,frictionAngleEstimate, estimateColor, time, terrainFrictionAngle(indexVector), trueColor)
%     ylabel('friction Angle (deg)')
%     xlabel('time (seconds)')
%     ylim([10 25])
% subplot(233)
%     plot(timeAugmented, n(1)*HL, '*', timeAugmented, n(2)*HL, '*', timeAugmented, n(3)*HL, '*', timeAugmented, n(4)*HL, '*', timeAugmented, n(5)*HL, '*',...
%         time,nEstimate, estimateColor, time, terrainn(indexVector), trueColor)
%     ylabel('n ')
%     xlabel('time (seconds)')
%     ylim([0.5 2])
% subplot(234)
%     plot(timeAugmented, keq(1)*HL, '*', timeAugmented, keq(2)*HL, '*', timeAugmented, keq(3)*HL, '*', timeAugmented, keq(4)*HL, '*', timeAugmented, keq(5)*HL, '*',...
%         time,keqEstimate, estimateColor, time, terrainkeq(indexVector), trueColor)
%     ylabel('keq ')
%     xlabel('time (seconds)')
%     ylim([100 600])
% subplot(235)
%     plot(timeAugmented, K(1)*HL, '*', timeAugmented, K(2)*HL, '*',timeAugmented, K(3)*HL, '*', timeAugmented, K(4)*HL, '*', timeAugmented, K(5)*HL, '*',...
%         time,KEstimate, estimateColor, time, terrainK, trueColor)
%     ylabel('K ')
%     xlabel('time (seconds)')
% subplot(236)
%     plot(timeAugmented, S(1)*HL, '*', timeAugmented, S(2)*HL, '*', timeAugmented, S(3)*HL, '*', timeAugmented, S(4)*HL, '*', timeAugmented, S(5)*HL, '*',...
%         time,SEstimate, estimateColor, time, terrainS(indexVector), trueColor)
%     ylabel('S')
%     xlabel('time (seconds)')

end