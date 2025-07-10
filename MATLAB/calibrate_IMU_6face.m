%% calibrate_IMU_6face.m  — *re‑v2: auto‑loads accelX/Y/Z style logs*
% 6‑position static calibration for a 3‑axis accelerometer.
% ─────────────────────────────────────────────────────────────────────────────
% NEW FEATURES (v2)
%   • Accepts either (i) an N×3 array **acc**, or (ii) a **.mat file name**
%     whose workspace variables are named  
%         accelX[, _1, _2 …]  accelY[… ]  accelZ[… ]
%   • Optional "sensorID" lets you pick which set ("", "_1", "_2", …) to
%     calibrate when several accelerometers are present in the file.
%   • No other assumptions about the file format—just those variable names.
%   • Keeps the previous interactive click‑selection for the six faces.
% 
% USAGE EXAMPLES
%   [S,bias,R] = calibrate_IMU_6face(accArray, fs);           % as before
%   [S,bias]   = calibrate_IMU_6face("accelCalibration.mat");% auto‑detect
%   [S,bias]   = calibrate_IMU_6face("accelCalibration.mat", "_2", 200);
%                                                    % pick *_2 sets, fs=200 Hz
% All outputs are the same:
%   S    – 1×3 per‑axis scale factors (multiply!)
%   bias – 1×3 offsets (subtract!)
%   R    – 3×3 mis‑alignment matrix (apply R*(acc-bias) )
% 
% ---------------------------------------------------------------------------
function [S, bias, R] = calibrate_IMU_6face(source, sensorID, fs)
% INPUTS
%   source    :   either N×3 numeric OR char/string MAT‑file name
%   sensorID  :   "" (default) , "_1", "_2", …  (ignored if source is array)
%   fs        :   sample rate in Hz (optional, default=1) – used only for plots
% ---------------------------------------------------------------------------
if nargin < 2, sensorID = "";             end
if nargin < 3, fs       = 1;               end

%% ---------------- 0. Load / assemble N×3 array ---------------------------
if isnumeric(source) && size(source,2)==3
    acc = double(source);
else
        if ~isfile(source), error("File not found: %s",source); end
    Sload = load(source);

    % ensure sensorID is a **character row vector** so that isfield returns
    % a single logical value (using a string array here makes isfield return
    % a vector and breaks the || / && tests)
    if nargin < 2 || isempty(sensorID)
        sensorID = '';
    end
    sensorID = char(sensorID);

    wantX = ['accelX' sensorID];
    wantY = ['accelY' sensorID];
    wantZ = ['accelZ' sensorID];

    if ~isfield(Sload,wantX) || ~isfield(Sload,wantY) || ~isfield(Sload,wantZ)
        error("Variables %s, %s, %s not found in %s",wantX,wantY,wantZ,source);
    end

    acc = double([ Sload.(wantX)(:), Sload.(wantY)(:), Sload.(wantZ)(:) ]);
    clear Sload
end
N = size(acc,1);

%% ------------ 1. Plot magnitude to pick six static faces ----------------
mag = vecnorm(acc,2,2);
figure('Name','6‑face calibration');
plot((0:N-1)/fs, mag, 'DisplayName','|a|'); grid on; hold on; legend show
xlabel('Time (s)'); ylabel('|a| (m/s^2)');
title({'Click START & END of each static face (six pairs)','Press <Enter> when done'});

idxPairs = zeros(6,2);
for k = 1:6
    [x,~] = ginput(2);           % two clicks → start & end
    idx   = round(x(:)'*fs) + 1; % seconds → sample index (1‑based)
    idx   = max(1, min(N, idx)); % clamp
    idx   = sort(idx);           % ensure ascending
    while diff(idx)==0
        warning('Face %d: zero‑length selection, pick again.',k);
        [x,~]=ginput(2);
        idx   = round(x(:)'*fs)+1;
        idx   = max(1,min(N,idx));
        idx   = sort(idx);
    end
    idxPairs(k,:) = idx;
    plot(idx/fs, mag(idx), 'rx');           % visual marker
end
hold off; pause(0.5); close(gcf);

%% ------------- 2. Mean acceleration vector for each face ---------------
faceMean = zeros(6,3);
for k = 1:6
    range = idxPairs(k,1):idxPairs(k,2);
    faceMean(k,:) = mean(acc(range,:),1);
end

%% ------------- 3. Solve bias & scale by least‑squares ------------------
g = 9.80665;
A = [];    % will become 18×6 (6 faces × 3 axes)
b = [];
for k = 1:6
    v = faceMean(k,:)';                  % 3×1

    % dominant axis → index & sign
    [~, idx] = max(abs(v));
    e        = zeros(3,1);               % expected gravity direction
    e(idx)   =  sign(v(idx));            %  +1 or -1 on dominant axis

    A = [A; diag(v) , -eye(3)];          % [diag(v) | -I]  (3×6 block)
    b = [b; g * e];                      %  (3×1)
end

x    = A \ b;                            % least‑squares solution (6×1)
S    = x(1:3)';                  % multiplicative scale factors
bias = x(4:6)';

%% -------- 4. Mis-alignment from covariance ellipsoid -------------
X = (acc - bias) .* S;          % pre-scaled, de-biased samples
C = (X' * X) / N;
[U,~] = eig(C);                 % col-vectors = principal axes

% --- (a) force a right-handed basis -------------------------------
if det(U) < 0, U(:,3) = -U(:,3);  end          % flip weakest axis

% --- (b) put the columns in X,Y,Z order ---------------------------
% find which eigen-vector aligns best with each *sensor* axis
[~,idx]   = max(abs(U),[],1);     % 1×3 :  row of max |component|
[~,perm]  = sort(idx);            %     :  desired order [1 2 3]
U         = U(:,perm);

% --- (c) make diagonal entries positive (optional but nice) -------
for k = 1:3
    if U(k,k) < 0
        U(:,k) = -U(:,k);
        S(k)   = -S(k);           % keep overall gain unchanged
    end
end

R = U.';                          % rows = sensor axes in world frame
for k = 1:3          % k = X,Y,Z
    if R(k,k) < 0    % row k points the wrong way
        R(k,:) = -R(k,:);
        S(k)   = -S(k);   % keep overall mapping identical
    end
end

%% ------------- 5. Show results & save ----------------------------------
fprintf('\nSix‑face calibration results (%s):\n', datestr(now));
fprintf('  Bias   : [%.4f  %.4f  %.4f]  m/s^2\n', bias);
fprintf('  Scale  : [%.6f %.6f %.6f]\n', S);
fprintf('  R (rows) =\n'); disp(R);

[outPath,~,~] = fileparts(mfilename('fullpath'));
savename = fullfile(outPath, sprintf('IMU_calibration%s.mat',sensorID));
save(savename,'S','bias','R');
fprintf('Saved → %s\n', savename);
end
