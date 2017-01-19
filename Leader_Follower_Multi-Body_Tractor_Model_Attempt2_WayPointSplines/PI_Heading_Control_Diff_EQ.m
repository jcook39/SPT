function [steerPumpCmdPI] = PI_Heading_Control_Diff_EQ(controller, headingError, timeStepNo)

% --------------------- Unpack Needed Parameters --------------------------
% Controller Parameters
hSysConD = controller.hSysConD;
hSysConDTFnum = hSysConD.num{1};
hSysConDTFden = hSysConD.den{1};
%fprintf('a = %f %f %f \n',hSysConDTFden)
%fprintf('b = %f %f %f \n',hSysConDTFnum)

b0 = hSysConDTFnum(1,1);
b1 = hSysConDTFnum(1,2);

a0 = hSysConDTFden(1,1);
a1 = hSysConDTFden(1,2);

% Look up past error values and inputs
ek = headingError;
if (timeStepNo-1) <= 0
    ekm1 = 0;
    ukm1 = 0;
else
    ekm1 = controller.headingError(timeStepNo-1,1);
    ukm1 = controller.steerPumpCmdPI(timeStepNo-1,1);
end
      

% ------------------- Compute Control Input -------------------------------
steerPumpCmdPI = (1/a0)*(-ukm1*a1 + b0*ek + b1*ekm1);

end