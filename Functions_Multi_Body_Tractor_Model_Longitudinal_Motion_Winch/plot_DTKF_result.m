function plot_DTKF_result(structDTKF, structRBE, nConstantTerrain, nConstantMT865, nTimeStep, timeStepS, figureNo)

% --------------------- Estimated State Vector ----------------------------
% xHat = [vHat omegaHat F_T,NetHat F_T,NetHatDot tau_ResHat tau_ResHatDot 
%           R_SHat R_SHatDot].';
%
% --------------------- True State Vector ----------------------------
% x = [v omega F_T,Net F_T,NetDot tau_Res tau_ResDot R_S R_SDot].';
%
% -------------------- Measurement Vector ---------------------------------
% y = [vDot v omega R_S].'


% ---------------- Create time Vector for plotting ------------------------
indexVector = 1:nTimeStep;
timeVector = (indexVector-1)*timeStepS;

% ----------------------- Unpack DTKF structure ---------------------------
x = structDTKF.x;
xHatPlus = structDTKF.xHatPlus;
xError = structDTKF.xError;
y = structDTKF.y;

smoothedvHat = structDTKF.smoothedvHat;

% ----------------------- Unpack Tractor Parameters -----------------------
rollingRadiusM = nConstantMT865.rollingRadiusM;
massTractorKG = nConstantMT865.massTractorKG;
massSledKG = nConstantMT865.massSledKG;
RsledN = nConstantMT865.RsledN;
gMPS2 = nConstantMT865.gMPS2;
resCoeff = nConstantMT865.resCoeff;


% ------------------------------ Plots ------------------------------------
lineWidthSize = 2;

trueColor = 'b';
estimateColor = 'r';
measureColor = 'k.';
smoothColor = 'g';

figure(figureNo)
set(gcf,'numbertitle','off','name','Discrete Time Kalman Filter Results')
% Speed
subplot(251)
    h = plot(timeVector,x(1,indexVector),trueColor,timeVector,xHatPlus(1,indexVector),estimateColor,...
        timeVector,y(2,indexVector),measureColor, timeVector, smoothedvHat, smoothColor);
    set(h(1),'linewidth',lineWidthSize)
    set(h(4),'linewidth',lineWidthSize)
    ylabel('Vehicle Speed')
    legend('True Value','Estimated Value','Measured Value','Location','SouthEast')
subplot(256)
    semilogy(timeVector,abs(xError(1,indexVector)),'r')
    ylabel('Vehicle Speed Estimate Error')
% Driver Speed
subplot(252)
    h = plot(timeVector,x(2,indexVector),trueColor,timeVector,xHatPlus(2,indexVector),estimateColor,timeVector,y(3,indexVector),measureColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Driver Speed')
subplot(257)
    semilogy(timeVector,abs(xError(1,indexVector)),'r')
    ylabel('Driver Speed Estimate Error')
%F_T,Net
subplot(253)
    h = plot(timeVector,x(3,indexVector),trueColor,timeVector,xHatPlus(3,indexVector),estimateColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Net Track Force')
subplot(258)
    semilogy(timeVector,abs(xError(3,indexVector)),'r')
    ylabel('Net Track Force Estimate Error')
% tauRes
subplot(254)
    h = plot(timeVector,x(5,indexVector),trueColor,timeVector,xHatPlus(5,indexVector),estimateColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Resistive Torque')
subplot(259)
    semilogy(timeVector,abs(xError(5,indexVector)),'r')
    ylabel('Resistive Torque Estimate Error')
% R_S
subplot(255)
    h = plot(timeVector,x(7,indexVector),trueColor,timeVector,xHatPlus(7,indexVector),estimateColor,timeVector,y(4,indexVector),measureColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Drawbar Load Estimate Error')
subplot(2,5,10)
    semilogy(timeVector,abs(xError(7,indexVector)),'r')
    ylabel('Drawbar Load Estimate Error')


resistanceSledEstimateN = xHatPlus(7,indexVector) - massSledKG*((xHatPlus(3,indexVector) - xHatPlus(7,indexVector))/massTractorKG);     
resistanceCoefficientEstimate = resistanceSledEstimateN./(massSledKG*gMPS2);

slipVectorEstimate = 100*(1-(xHatPlus(1,indexVector)./(rollingRadiusM*xHatPlus(2,indexVector))));
slipVectorTrue = 100*(1-(x(1,indexVector)./(rollingRadiusM*x(2,indexVector))));
slipVectorMeasure = 100*(1-(y(2,indexVector)./(rollingRadiusM*y(3,indexVector))));
figure(figureNo+1)
set(gcf,'numbertitle','off','name','Slip-Drawbar Force Estimation')
subplot(221)
    plot(timeVector,slipVectorTrue,trueColor,timeVector,slipVectorEstimate,estimateColor,timeVector,slipVectorMeasure,measureColor)
    set(h(1),'linewidth',lineWidthSize)
    xlabel('time (seconds)')
    ylabel('slip')
    ylim([0 100])
    legend('True Value','Estimated Value','Measured Value')
    grid on
subplot(222)
    plot3(timeVector,slipVectorEstimate,xHatPlus(7,indexVector),estimateColor)
    zlabel('Estimated Drawbar Force')
    ylabel('Estimated Slip')
    xlabel('time (seconds)')
    ylim([0 100])
subplot(223)
    plot(timeVector,resistanceSledEstimateN,estimateColor,timeVector,RsledN*ones(numel(timeVector)),trueColor)
    xlabel('time (seconds)')
    ylabel('Resistance Sled Estimate (N)')
subplot(224)
    plot(timeVector,resistanceCoefficientEstimate,estimateColor,timeVector,resCoeff*ones(numel(timeVector),1),trueColor)
    xlabel('time seconds')
    ylabel('Sled Resistance Coefficient')
    
fontLabel = 18;    
figure(figureNo+2)
subplot(321)
    h = plot(timeVector,x(1,indexVector),trueColor,timeVector,xHatPlus(1,indexVector),estimateColor,...
        timeVector,y(2,indexVector),measureColor, timeVector, smoothedvHat, smoothColor);
    set(h(1),'linewidth',lineWidthSize)
    set(h(4),'linewidth',lineWidthSize)
    ylabel('Vehicle Speed, $v_T$ (m/s)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    legend('True Value','Estimated Value','Measured Value','Location','NorthEast')
    grid on
subplot(322)    
    h = plot(timeVector,x(2,indexVector),trueColor,timeVector,xHatPlus(2,indexVector),estimateColor,timeVector,y(3,indexVector),measureColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Driver Speed, $\dot{\varphi}$ (rad/s)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    ylim([0 4])
    grid on
subplot(323)    
    h = plot(timeVector,slipVectorTrue,trueColor,timeVector,slipVectorEstimate,estimateColor,timeVector,slipVectorMeasure,measureColor)
    set(h(1),'linewidth',lineWidthSize)
    xlabel('time (seconds)')
    ylabel('slip ratio, i (\%)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    ylim([-10 100])
    %legend('True Value','Estimated Value','Measured Value')
    grid on
subplot(324)    
    h = plot(timeVector,x(3,indexVector),trueColor,timeVector,xHatPlus(3,indexVector),estimateColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Net Track Force, $F_{Net}$ (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    grid on
subplot(325)    
    h = plot(timeVector,x(5,indexVector),trueColor,timeVector,xHatPlus(5,indexVector),estimateColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Resistive Torque, $\tau_{Res}$ (Nm)','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    ylim([0 1.5e5])
    grid on
subplot(326)
    h = plot(timeVector,x(7,indexVector),trueColor,timeVector,xHatPlus(7,indexVector),estimateColor,timeVector,y(4,indexVector),measureColor);
    set(h(1),'linewidth',lineWidthSize)
    ylabel('Drawbar Load Estimate, $DB$ (N) ','interpreter','latex','fontsize',fontLabel)
    xlabel('time (seconds)','interpreter','latex','fontsize',fontLabel)
    grid on
    
    
end