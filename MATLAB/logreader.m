% logreader.m
% Use this script to read data from your micro SD card

clear;
clf;

filenum = '050'; % file number for the data you want to read
infofile = strcat('inf', filenum, '.txt');
datafile = strcat('log', filenum, '.bin');

%% map from datatype to length in bytes
dataSizes.('float') = 4;
dataSizes.('ulong') = 4;
dataSizes.('int') = 4;
dataSizes.('int32') = 4;
dataSizes.('uint8') = 1;
dataSizes.('uint16') = 2;
dataSizes.('char') = 1;
dataSizes.('bool') = 1;

%% read from info file to get log file structure
fileID = fopen(infofile);
items = textscan(fileID,'%s','Delimiter',',','EndOfLine','\r\n');
fclose(fileID);
[ncols,~] = size(items{1});
ncols = ncols/2;
varNames = items{1}(1:ncols)';
varTypes = items{1}(ncols+1:end)';
varLengths = zeros(size(varTypes));
colLength = 256;
for i = 1:numel(varTypes)
    varLengths(i) = dataSizes.(varTypes{i});
end
R = cell(1,numel(varNames));

%% read column-by-column from datafile
fid = fopen(datafile,'rb');
for i=1:numel(varTypes)
    %# seek to the first field of the first record
    fseek(fid, sum(varLengths(1:i-1)), 'bof');
    
    %# % read column with specified format, skipping required number of bytes
    R{i} = fread(fid, Inf, ['*' varTypes{i}], colLength-varLengths(i));
    eval(strcat(varNames{i},'=','R{',num2str(i),'};'));
end
fclose(fid);

%% Read IMU Data from csv
imuDataFile = strcat('csv','log','11','.csv');
tbl = readtable(imuDataFile);
accelX_1 = tbl.AccelX1;
accelY_1 = tbl.AccelY1;
accelZ_1 = tbl.AccelZ1;
accelX_2 = tbl.AccelX2;
accelY_2 = tbl.AccelY2;
accelZ_2 = tbl.AccelZ2;
gyroX1 = tbl.GyroX1;
gyroY1 = tbl.GyroY1;
gyroZ1 = tbl.GyroZ1;
gyroX2 = tbl.GyroX2;
gyroY2 = tbl.GyroY2;
gyroZ2 = tbl.GyroZ2;


start = 264;
last = 710;

accelY = accelY(start:last);
accelY_1 = accelY_1(start:last);

accelY_2 = accelY_2(start:last);
% avgY = (accelY + accelY_2)/2;
offsetY = mean(accelY - accelY_1);
accelY_1 = accelY_1 + offsetY;
fprintf("Average Difference between IMU 1 and IMU 2: %.3f\n", offsetY);

accelZ = accelZ(start:last);
accelZ_1 = accelZ_1(start:last);

accelZ_2 = accelZ_2(start:last);
avgZ = (accelZ_1 + accelZ_2)/2;
offsetZ = mean(avgZ - accelZ);
accelZ = accelZ + offsetY;
% fprintf("Average Difference board and IMU12 Z: %.3f\n", offset);

% hold on
% plot(accelY, 'DisplayName', 'Board IMU X');
% plot(accelY_1 + offsetY, 'DisplayName', 'IMU 1 X');
% plot(accelY_2, 'DisplayName', 'IMU 2 X');
% legend();

% 
accelX = accelX(start:last);
magX = magX(start:last);
magY = magY(start:last);
magZ = magZ(start:last);

accelX_1 = accelX_1(start:last);

accelX_2 = accelX_2(start:last);

gyroX1 = gyroX1(start:last);
gyroY1 = gyroY1(start:last);
gyroZ1 = gyroZ1(start:last);

gyroX2 = gyroX2(start:last);
gyroY2 = gyroY2(start:last);
gyroZ2 = gyroZ2(start:last);
motorB = motorB(start:last);

z = z(start:last);
depth_des = depth_des(start:last);

save secondPierRun.mat accelX accelY accelZ accelX_1 accelY_1 accelZ_1 accelX_2 accelY_2 ...
    accelZ_2 gyroX1 gyroY1 gyroZ1 gyroX2 gyroY2 gyroZ2 magX magY magZ z depth_des motorB;
% % % N = length(accelX_1);
% t = (0:N-1) / 10.1;
% 
% tblBoard       = timetable(t, accelX, accelY, accelZ, ...
%                            magX, magY, magZ,       z);
% tblOther1      = timetable(t(1:end-1), ...
%                            accelX_1, accelY_1, accelZ_1, ...
%                            gyroX1,   gyroY1,   gyroZ1);
% tblOther2      = timetable(t(1:end-1), ...
%                            accelX_2, accelY_2, accelZ_2, ...
%                            gyroX2,   gyroY2,   gyroZ2);
% 
% raw            = synchronize(tblBoard, tblOther1, tblOther2, 'firstvalue');

% acc0  = [raw.accelX raw.accelY raw.accelZ];             % Board IMU
% acc1  = [raw.accelX_1 raw.accelY_1 raw.accelZ_1];       % Other IMU #1
% acc2  = [raw.accelX_2 raw.accelY_2 raw.accelZ_2];       % Other IMU #2
% 
% bgyro1 = [raw.gyroX1 raw.gyroY1 raw.gyroZ1];
% gyro2 = [raw.gyroX2 raw.gyroY2 raw.gyroZ2];
% 
% 
% %% Process your data here
% pressureData = A00;
% resistorData = A01;
% %% Acceleration stuff
% accA = [accelX   accelY   accelZ  ];   % N×3  (Board IMU)
% accB = [accelX_1 accelY_1 accelZ_1];   % N×3  (sensor 1)
% accC = [accelX_2 accelY_2 accelZ_2];   % N×3  (sensor 2) 
% gyrA = [gyroX1 gyroY1 gyroZ1];         % N×3
% gyrB = [gyroX2 gyroY2 gyroZ2];         % N×3
% 
% % -------- magnetometer & depth --------------------------
% mag  = [magX  magY  magZ];             % N×3
% z    =  z;                             % N×1  (depth or altitude)
% 
% t    = (0:length(z)-1).' * 0.099;      % ≈ sample interval 0.099 s → 10.1 Hz
% 
% g0      = 9.80665;          % m s⁻²
% accNorm = vecnorm(accA,2,2);   % pick one sensor for stationarity test
% gyrNorm = vecnorm(gyrA,2,2);
% 
% isStatic = abs(accNorm-g0) < 0.15 & gyrNorm < deg2rad(1);  % tweak thresholds
% 
% accA = accA - biasAcc(1,:);
% accB = accB - biasAcc(2,:);
% accC = accC - biasAcc(3,:);
% gyrA = gyrA - biasGyr(1,:);
% gyrB = gyrB - biasGyr(2,:);
% 
% 
% % --- accelerometer & gyro biases -------------------------
% biasAcc = [mean(accA(isStatic,:));
%            mean(accB(isStatic,:));
%            mean(accC(isStatic,:))];      % 3×3  (row per sensor)
% 
% biasGyr = [mean(gyrA(isStatic,:));
%            mean(gyrB(isStatic,:))];      % 2×3
% % remove effects of gravity
% 
% averageX = (accelX + accelX_1 + accelX_2) / 3;
% averageY = (accelY + accelY_1 + accelY_2) / 3;
% averageZ = (accelZ + accelZ_1 + accelZ_2) / 3;
% 
% 
% % 1. Rotate all the data into the same reference frame
% combinedAccelX = [accelX, accelX_1, accelX_2];
% combinedAccelY = [accelY, accelY_1, accelY_2];
% combinedAccelZ = [accelZ, accelZ_1, accelZ_2];
% % 2. Fuse data using weighted averaging and outlier rejection
% w = [0.25,0.375,0.375];
% %weightedAccelX = 0.25 * accelX + 0.75 * specialIMUX;
% n = size(combinedAccelZ,1);

% function fused = fuse_with_outlier_rejection(A, w, thresh)
% % A:     n×3 matrix of acceleration data
% % w:     1×3 vector of weights (must sum to 1)
% % thresh: number of std devs for outlier rejection (e.g., 1)
% 
%     if nargin < 3
%         thresh = 1; % default threshold
%     end
% 
%     n = size(A,1);
%     fused = zeros(n,1);
% 
%     for i = 1:n
%         row = A(i,:);
%         row_mean = mean(row);
%         row_std = std(row);
% 
%         % Reject outliers based on threshold
%         inliers = abs(row - row_mean) < thresh * row_std;
% 
%         if any(inliers)
%             valid_weights = w(inliers);
%             valid_weights = valid_weights / sum(valid_weights); % normalize
%             fused(i) = sum(row(inliers) .* valid_weights);
%         else
%             fused(i) = row_mean;  % fallback if all are outliers
%         end
%     end
% end
% 
% fusedAccelX = fuse_with_outlier_rejection(combinedAccelX,w,1);
% fusedAccelY = fuse_with_outlier_rejection(combinedAccelY,w,1);
% fusedAccelZ = fuse_with_outlier_rejection(combinedAccelZ,w,1);
% 
% 
% % 3. Apply low-pass filter to fused data
% filteredAccelX = medfilt1(fusedAccelX,3);
% filteredAccelY = medfilt1(fusedAccelY,3);
% filteredAccelZ = medfilt1(fusedAccelZ,3);

% posZ = cumtrapz(cumtrapz(filteredAccelX));
% % 4. Analyze acceleration data. Compare raw data to fused and filtered data
% t = (0:n-1) / 10.1;  % time vector in seconds
% figure(1);
% clf
% hold on
% plot(t, filteredAccelZ, '-r', 'LineWidth',3);
% plot(t, averageZ, '-g', 'LineWidth', 2);
% legend('Filtered Acceleration', 'Averaged Acceleration');
% % plot(t, accelZ, '-og', 'LineWidth', 1);
% % plot(t, accelZ_1, '-ob', 'LineWidth', 1);
% % plot(t, accelZ_2, '-om', 'LineWidth', 1);
% % legend('Fused Acceleration', 'Board Acceleration', 'IMU 1 Acceleration', 'IMU 2 Acceleration')
% xlabel("Time [s]");
% ylabel("Acceleration in Z Direction [m/s^2]");
% title("Plot of Raw and Fused Acceleration Data vs Time");
% 
% figure(2);
% clf
% hold on
% plot(t, filteredAccelY, '-r', 'LineWidth',2);
% plot(t, averageY, '-g', 'LineWidth', 2);
% legend('Filtered Acceleration', 'Averaged Acceleration');
% % plot(t, accelY, '--g', 'LineWidth', 1);
% % plot(t, accelY_1, '--b', 'LineWidth', 1);
% % plot(t, accelY_2, '--m', 'LineWidth', 1);
% % legend('Fused Acceleration', 'Board Acceleration', 'IMU 1 Acceleration', 'IMU 2 Acceleration')
% xlabel("Time [s]");
% ylabel("Acceleration in Y Direction[m/s^2]");
% title("Plot of Raw and Fused Acceleration Data vs Time");
% 
% figure(3);
% clf
% hold on
% plot(t, filteredAccelX, '-r', 'LineWidth',2);
% plot(t, averageX, '-g', 'LineWidth', 2);
% legend('Filtered Acceleration', 'Averaged Acceleration');
% % plot(t, accelX, '-og', 'LineWidth', 1);
% % plot(t, accelX_1, '-ob', 'LineWidth', 1);
% % plot(t, accelX_2, '-om', 'LineWidth', 1);
% % legend('Filtered Acceleration', 'Board Acceleration', 'IMU 1 Acceleration', 'IMU 2 Acceleration');
% xlabel("Time [s]");
% ylabel("Acceleration in X Direction[m/s^2]");
% title("Plot of Raw and Fused Acceleration Data vs Time");
% 
% figure(4);
% clf;
% hold on
% %plot(t, posZ, 'r', 'LineWidth',1);
% plot(t, depth(1:n), 'g', 'LineWidth',1);
% plot(t, depth_des(1:n), 'b', 'LineWidth',1);