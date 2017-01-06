function slip = calculate_slip(groundSpeedMPS, trackSpeedRadPS, nConstantMT865)

% ---------------- Unpack Needed Tractor Parameters -----------------------
rollingRadiusM = nConstantMT865.rollingRadiusM;

% ---------------- Slip Calculation ---------------------------------------
if (groundSpeedMPS <= 0 ), slip = 0;
else slip = 1 - (groundSpeedMPS/(trackSpeedRadPS*rollingRadiusM)); 
    if slip < 0, slip = 0; end
    if slip > 1, slip = 1; end
end
slip = slip*100 + 1E-10;


end