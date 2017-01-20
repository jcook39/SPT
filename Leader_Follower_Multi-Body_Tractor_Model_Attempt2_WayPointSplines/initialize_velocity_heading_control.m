function controllerStruct = initialize_velocity_heading_control(Kpv, Kiv, Kph, Kih, kx, refDlong, refDlat, timeArray)

% Build controller structure
controllerStruct.Kpv = Kpv;
controllerStruct.Kiv = Kiv;
controllerStruct.Kph = Kph;
controllerStruct.Kih = Kih;
controllerStruct.kx = kx;
controllerStruct.refDlong = refDlong;
controllerStruct.refDlat = refDlat;

% Controller reference input storage
controllerStruct.headingRef = zeros(size(timeArray,1),1);
controllerStruct.speedRef = zeros(size(timeArray,1),1);

% Controller Error Values Storage
controllerStruct.errorx = zeros(size(timeArray,1),1);
controllerStruct.headingError = zeros(size(timeArray,1),1); 
controllerStruct.headingIntegratedError = zeros(size(timeArray,1),1); 
controllerStruct.speedError = zeros(size(timeArray,1),1);
controllerStruct.speedIntegratedError = zeros(size(timeArray,1),1);

% Controller Output Storage
controllerStruct.steerPumpCmd = zeros(size(timeArray,1),1);
controllerStruct.steerPumpCmdPI = zeros(size(timeArray,1),1);
controllerStruct.throttleCommand = zeros(size(timeArray,1),1);
controllerStruct.throttleCommandPI = zeros(size(timeArray,1),1);


% Build speed controller difference equation
vConC = pid(Kpv, Kiv);
vConSampleTime = 0.1; % 10 Hz;
vSysConC = tf(vConC);
vSysConD = c2d(vSysConC,vConSampleTime,'tustin');
controllerStruct.vSysConD = vSysConD; % structure for z transfer function for veloicty control

% Build heading controller difference equation
hConC = pid(Kph, Kih);
hConSampleTime = 0.1;
hSysConC = tf(hConC);
hSysConD = c2d(hSysConC, hConSampleTime, 'tustin');
controllerStruct.hSysConD = hSysConD;

end