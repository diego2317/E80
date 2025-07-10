function [mx, my, mz] = undoMagCalibration(magX, magY, magZ)
% undoMagCalibration removes soft-iron calibration and adds offsets back
%
% Inputs:
%   magX, magY, magZ : Nx1 vectors of calibrated magnetometer readings (µT)
%
% Outputs:
%   mx, my, mz : Nx1 vectors of uncalibrated magnetometer readings (µT)

    % 1. Build the soft-iron matrix (from firmware)
    Msoft = [ 0.0232  -0.0016  -0.0001 ;
              0        0.025    0.0017 ;
              0        0        0.0248 ];
    
    % 2. Invert it
    Minv = inv(Msoft);

    % 3. Firmware offsets (µT)
    bOff = [8.9059  34.1319  -31.3076];

    % 4. Remove soft-iron effect and add offsets back
    mCal = [magX magY magZ];       % Nx3 array
    m_uT = (Minv * mCal.').';      % remove soft-iron (still offset)
    m_uT_raw = m_uT + bOff;        % add offsets back

    % 5. Split into individual components
    mx = m_uT_raw(:,1);
    my = m_uT_raw(:,2);
    mz = m_uT_raw(:,3);
end
