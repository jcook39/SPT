function plot_control_performance(tractor,inputMat,controller,trajSmooth,nTimeStep,timeStepS)

% Create Time Vectors for plotting
indexVector = 1:nTimeStep;
timeVector = (indexVector-1)*timeStepS;

% Unpack Needed tractor parameters
thetaDot = zeros(nTimeStep,1);
for i = 1:nTimeStep
   X(i) = tractor(i).state(1);
   Y(i) = tractor(i).state(2);
   theta(i) = tractor(i).state(3);
   vx(i) = tractor(i).state(6);
   vy(i) = tractor(i).state(7);
   thetaDot(i) = tractor(i).state(8);
end
steerAngleDeg = inputMat(indexVector,3);
speed = sqrt(vx.^2 + vy.^2).';

% Unpack reference trajectory
yawRateCmdTraj = trajSmooth.yawRateCmdTraj;
gpsHeading = atan2(vy,vx) + theta;
curvature = yawRateCmdTraj./speed;
turningRadius = 1./curvature;

% Unpack controller performance and signals
yawRateError = controller.errorMatrix(1,:);
yawRateIntegratedError = controller.integratedErrorMatrix(1,:);
steerPumpCmd = controller.steerPumpCmd;

for i = 1:nTimeStep
    pivotPointBodyFixedFrame1(:,i) = [0 turningRadius(1) 0].';
    pivotPointGlobalFrame(:,i) = [X(i) Y(i) 0].'  + rotation_matrix_z(-gpsHeading(i))*pivotPointBodyFixedFrame1(:,i);
end

pivotPointGlobalFrame(isinf(pivotPointGlobalFrame)) = 0;
pivotPointGlobalFrame(isnan(pivotPointGlobalFrame)) = 0;

figure(15)
plot(pivotPointGlobalFrame(1,:),pivotPointGlobalFrame(2,:),'rx')

figure(11)
subplot(331)
    plot(timeVector,steerAngleDeg)
    ylabel('steerAngleCmd (deg) driver')
subplot(332)
    plot(timeVector,yawRateCmdTraj,timeVector,thetaDot)
    legend('yawRateCmdTraj', 'yawRate/thetaDot ')
subplot(333)
    plot(timeVector,yawRateError)
    ylabel('error signal')
subplot(334)    
    plot(timeVector,yawRateIntegratedError)
    ylabel('integrated error signal')
subplot(335)    
    plot(timeVector,steerPumpCmd)
    ylabel('steer Pump Cmd')
subplot(336)
    plot(timeVector,curvature)
    ylabel('path curvature ref')
subplot(337)
    plot(timeVector,pivotPointGlobalFrame(1,:))
    ylabel('pivot Point global Frame X')
subplot(338)
    plot(timeVector,pivotPointGlobalFrame(2,:))
    ylabel('pivotPoint global Frame Y')
        
end