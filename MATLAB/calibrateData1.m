function [acc, gyro, mag] = calibrateData1(source)

D = load('calibrationData.mat');
R = load(source); % Raw data goes here

Fs = 10.1; % Hz
N = numel(R.accelX); % # samples

%% Build raw matrices
acc{1}  = [R.accelX,   R.accelY,   R.accelZ]; % Board IMU
acc{2}  = [R.accelX_1, R.accelY_1, R.accelZ_1]; % Other IMU # 1
acc{3}  = [R.accelX_2, R.accelY_2, R.accelZ_2]; % Other IMU # 2

gyroRaw{1} = [R.gyroX1, R.gyroY1, R.gyroZ1];
gyroRaw{2} = [R.gyroX2, R.gyroY2, R.gyroZ2];


%% We need to "undo" the accidental calibration we did to magRaw
[mx, my, mz] = calibrationFunctions.undoMagCalibration(R.magX, R.magY, R.magZ);
magRaw = [mx, my, mz];

%% Now, we calibrate our sensors
% Calibrate accelerometers
% for k = 1:3
%     acc{k} = calibrationFunctions.accCal(accRaw{k}, D.cal.acc(k));
% end
% % flip sign of other IMU 1 X accel
% acc{1,2}(:,1) = -acc{1,2}(:,1);
% acc{1,2}(:,3) = -acc{1,2}(:,1);
% % Calibrate gyros
for k = 1:2
    gyro{k} = calibrationFunctions.gyroCal(gyroRaw{k}, D.cal.gyro(k));
end

% Calibrate magnetometer
mag = calibrationFunctions.magCal(magRaw, D.cal.mag);


end