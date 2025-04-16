% logreader.m
% Use this script to read data from your micro SD card

clear;
clf;

filenum = '063'; % file number for the data you want to read
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

%% Process your data here
pressureData = A00;
resistorData = A01;

%% Acceleration stuff

averageX = (accelX + accelX_1 + accelX_2) / 3;
averageY = (accelY + accelY_1 + accelY_2) / 3;
averageZ = (accelZ + accelZ_1 + accelZ_2) / 3;


% 1. Rotate all the data into the same reference frame
combinedAccelX = [accelX, accelX_1, accelX_2];
combinedAccelY = [accelY, accelY_1, accelY_2];
combinedAccelZ = [accelZ, accelZ_1, accelZ_2];
% 2. Fuse data using weighted averaging and outlier rejection
w = [0.25,0.375,0.375];
%weightedAccelX = 0.25 * accelX + 0.75 * specialIMUX;
n = size(combinedAccelZ,1);

function fused = fuse_with_outlier_rejection(A, w, thresh)
% A:     n×3 matrix of acceleration data
% w:     1×3 vector of weights (must sum to 1)
% thresh: number of std devs for outlier rejection (e.g., 1)

    if nargin < 3
        thresh = 1; % default threshold
    end

    n = size(A,1);
    fused = zeros(n,1);

    for i = 1:n
        row = A(i,:);
        row_mean = mean(row);
        row_std = std(row);

        % Reject outliers based on threshold
        inliers = abs(row - row_mean) < thresh * row_std;

        if any(inliers)
            valid_weights = w(inliers);
            valid_weights = valid_weights / sum(valid_weights); % normalize
            fused(i) = sum(row(inliers) .* valid_weights);
        else
            fused(i) = row_mean;  % fallback if all are outliers
        end
    end
end

fusedAccelX = fuse_with_outlier_rejection(combinedAccelX,w,1);
fusedAccelY = fuse_with_outlier_rejection(combinedAccelY,w,1);
fusedAccelZ = fuse_with_outlier_rejection(combinedAccelZ,w,1);


% 3. Apply low-pass filter to fused data
filteredAccelX = medfilt1(fusedAccelX,3);
filteredAccelY = medfilt1(fusedAccelY,3);
filteredAccelZ = medfilt1(fusedAccelZ,3);
% 4. Analyze acceleration data. Compare raw data to fused and filtered data
t = (0:n-1) / 10.1;  % time vector in seconds
figure(1);
clf
hold on
plot(t, filteredAccelZ, '-r', 'LineWidth',3);
plot(t, averageZ, '-g', 'LineWidth', 2);
legend('Filtered Acceleration', 'Averaged Acceleration');
% plot(t, accelZ, '-og', 'LineWidth', 1);
% plot(t, accelZ_1, '-ob', 'LineWidth', 1);
% plot(t, accelZ_2, '-om', 'LineWidth', 1);
% legend('Fused Acceleration', 'Board Acceleration', 'IMU 1 Acceleration', 'IMU 2 Acceleration')
xlabel("Time [s]");
ylabel("Acceleration in Z Direction [m/s^2]");
title("Plot of Raw and Fused Acceleration Data vs Time");

figure(2);
clf
hold on
plot(t, filteredAccelY, '-r', 'LineWidth',2);
plot(t, averageY, '-g', 'LineWidth', 2);
legend('Filtered Acceleration', 'Averaged Acceleration');
% plot(t, accelY, '--g', 'LineWidth', 1);
% plot(t, accelY_1, '--b', 'LineWidth', 1);
% plot(t, accelY_2, '--m', 'LineWidth', 1);
% legend('Fused Acceleration', 'Board Acceleration', 'IMU 1 Acceleration', 'IMU 2 Acceleration')
xlabel("Time [s]");
ylabel("Acceleration in Y Direction[m/s^2]");
title("Plot of Raw and Fused Acceleration Data vs Time");

figure(3);
clf
hold on
plot(t, filteredAccelX, '-r', 'LineWidth',2);
plot(t, averageX, '-g', 'LineWidth', 2);
legend('Filtered Acceleration', 'Averaged Acceleration');
% plot(t, accelX, '-og', 'LineWidth', 1);
% plot(t, accelX_1, '-ob', 'LineWidth', 1);
% plot(t, accelX_2, '-om', 'LineWidth', 1);
% legend('Filtered Acceleration', 'Board Acceleration', 'IMU 1 Acceleration', 'IMU 2 Acceleration');
xlabel("Time [s]");
ylabel("Acceleration in X Direction[m/s^2]");
title("Plot of Raw and Fused Acceleration Data vs Time");