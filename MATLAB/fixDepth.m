function corrected_depth = fixDepth(voltage_incorrect)
    % Parameters from your calibration curve
    V0 = 2.52;
    sensitivity = -1.09;
    
    % Reverse the incorrect transformation
    voltage_corrected = (voltage_incorrect - V0) / sensitivity;
    
    % Now apply the correct voltage-to-depth conversion
    corrected_depth = voltage_corrected*-0.918 + 2.31;
end