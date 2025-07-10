clear
clf
S   = load('gyroCalibration.mat');         % change to your file
gX1 = S.gyroX_1(:); gY1 = S.gyroY_1(:); gZ1 = S.gyroZ_1(:);
Fs = 10.1;
thr = 3; % deg/s
% ---------- detect still block ----------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gyroRaw   : N×3 matrix  [gX gY gZ]  in rad/s
% Fs        : sample rate (Hz)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

gyroRaw = [gX1 gY1 gZ1];           % N×3  (your vectors)

%% ---------- pass 0 : very loose threshold ------------------------------
thr0 = 1.0;                        % 1 rad/s  (~57 °/s)
still0 = vecnorm(gyroRaw,2,2) < thr0;

% Use the *longest contiguous* still block
blockStarts = find(diff([0; still0])== 1);
blockEnds   = find(diff([still0; 0])==-1);
[~,k]       = max(blockEnds-blockStarts);
idx0        = blockStarts(k):blockEnds(k);

bias0 = mean(gyroRaw(idx0,:),1);   % coarse bias

%% ---------- pass 1 : refine around the coarse bias ---------------------
gyro1 = gyroRaw - bias0;           % bias-removed
thr1  = 0.01;                      % 0.03 rad/s  (~1.7 °/s)
still1 = vecnorm(gyro1,2,2) < thr1;

if ~any(still1)
    warning('No samples below %.3f rad/s — fallback to coarse bias.', thr1);
    bias = bias0;
else
    bias = mean(gyroRaw(still1,:),1);   % fine bias in **raw** units
end

fprintf('Gyro bias  [rad/s]  = %.6f  %.6f  %.6f \n', bias);

gyroClean = gyroRaw - bias;        % ready for Allan-variance, EKF, etc.


%% ---------- Allan variance ----------
N           = size(gyroClean,1);
maxCluster  = floor(N/2);                    % longest allowed cluster
clusterLen  = 2.^(0:floor(log2(maxCluster)));% 1,2,4,8,… up to N/2
[avar, tau] = allanvar(gyroClean, 'octave', Fs);

%% ---------- plot Allan deviation (σ) ----------
loglog(tau, sqrt(avar(:,3)));   % Z-axis (column 3)
grid on
xlabel('\tau (s)'); ylabel('\sigma_{ADEV} (rad/s)')
title('Gyro Z-axis Allan deviation')
