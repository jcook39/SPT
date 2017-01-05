function plot_heading_control_performance(controller,wayPointFlagMatNew,timeStepS,nTimeStep)

% Create Time Vectors for plotting
indexVector = 1:nTimeStep;
timeVector = (indexVector-1)*timeStepS;

headingRef = controller.headingRef(indexVector);
headingError = controller.errorMatrix(1,indexVector);
headingIntegratedError = controller.integratedErrorMatrix(1,indexVector);
steerPumpCmd = controller.steerPumpCmd;


figure(11)
subplot(231)
    plot(timeVector,headingRef*(180/pi))
    ylabel('headingRef (deg)')
subplot(232)
    plot(timeVector,headingError*(180/pi))
    ylabel(' heading error (deg)')
subplot(233)
    plot(timeVector,headingIntegratedError*(180/pi))
    ylabel(' heading integrated error (deg) ')
subplot(234)
    plot(timeVector,steerPumpCmd)
    ylabel(' steerPumpCmd ')
subplot(235)
    plot(timeVector,wayPointFlagMatNew(indexVector,1))
    ylabel('X WayPoint')
subplot(236)
    plot(timeVector,wayPointFlagMatNew(indexVector,2))
    ylabel('Y WayPoint')
    
end