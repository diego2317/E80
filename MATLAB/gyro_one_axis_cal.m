function [S_axis, epsOther] = gyro_one_axis_cal(gyroXYZ, axis, nTurns)
% gyroXYZ : N×3 rad/s  [ωx ωy ωz]
% axis    : 'x' | 'y' | 'z'  – the axis you intentionally rotated about
% tRev    : seconds for ONE full 360° turn
% nTurns  : number of whole turns in *seg*
% Fs      : sample-rate (Hz)  → dt = 1/Fs
%
% Returns
%   S_axis   : scale-factor for the spun axis
%   epsOther : 1×2 vector  [ε_leak1  ε_leak2]  (rad)  ⇢ use in R-matrix:
%                spin X →  [ε_xy  ε_xz]
%                spin Y →  [ε_yx  ε_yz]
%                spin Z →  [ε_zx  ε_zy]

% ---------------- pick columns -------------------------------------------
ax      = lower(axis);
colMain = find('xyz'==ax);                     % 1,2,3
if isempty(colMain), error('axis must be x, y, or z'); end
colsOther = setdiff(1:3,colMain);             % the other two axes

% ---------------- crop segment (if not already) --------------------------
seg = gyroXYZ(140:422);                                % <-- your own cropping

% ---------------- scale factor (angle integration) -----------------------
dt          = 0.05;
theta_meas  = sum(seg(:,colMain))*dt;         % ∫ω_main dt
theta_true  = 2*pi*nTurns;                    % known rad
S_axis      = theta_true / theta_meas;        % >0 if sign matches

% ---------------- leakage EPSILONS (small angles, rad) -------------------
epsOther = zeros(1,2);
for k = 1:2
    theta_leak = sum(seg(:,colsOther(k))) * dt;   % ∫ω_leak dt
    epsOther(k)= abs(theta_leak) / abs(theta_meas);% ratio ≈ tan(ε) ≈ ε
end
% convert to degrees if you prefer:  epsOtherDeg = rad2deg(epsOther);

% ---------------- print summary ------------------------------------------
labs = 'xyz';
fprintf('Spin about %c:  S = %.5f\n', labs(colMain), S_axis);
fprintf('            leakage to %c = %.4f rad  (%.2f°)\n', ...
        labs(colsOther(1)), epsOther(1), rad2deg(epsOther(1)));
fprintf('            leakage to %c = %.4f rad  (%.2f°)\n', ...
        labs(colsOther(2)), epsOther(2), rad2deg(epsOther(2)));
end
