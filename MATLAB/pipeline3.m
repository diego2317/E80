%% High-level pipeline for sensor calibration, fusion, attitude filter, linear accel removal
clear
clf
source = 'firstPierRunNew.mat'; % Raw Data filename
Fs = 10.1;
dt = 1/Fs;

% This one doesn't calibrate data
[acc, gyro, mag] = calibrateDataButDont(source);
fusedAcc = FuseData.fuseAccel(acc, Fs);
fusedGyro = FuseData.fuseGyro(gyro);
R = load(source);
depth = R.z;
depth_des = R.depth_des;
uV = R.motorB;
N = size(depth);

N = N(1);

%% Tilt-compass orientation from accel + mag
% Heading with tilt compensation
ax = fusedAcc(:,1);
ay = fusedAcc(:,2);
az = fusedAcc(:,3);
roll = atan2(ay, az);
pitch = atan2(-ax, sqrt(ay.^2 + az.^2));
mx = mag(:,1);  my = mag(:,2);  mz = mag(:,3);

% pre‐compute trig
cr = cos(roll);  sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);

% tilt‐compensated horizontal field
Xh = mx.*cp + my.*sr.*sp + mz.*cr.*sp;
Yh =   my.*cr      - mz.*sr;


t = (0:numel(roll)-1)/Fs;

[roll, pitch, yaw] = computeOrientationMadgwick(fusedAcc, fusedGyro, mag);
%plot(yaw)
% assume yaw is in degrees
%yaw_cont = rad2deg( unwrap( deg2rad(yaw) ) );
yaw = atan2(Yh, Xh);
%yaw = rad2deg(yaw);
%% Yaw drift complimentary filter
alpha = 0.85;      % gyro trust factor

yaw_cf = zeros(size(yaw));
for k = 2:length(yaw)
  dy       = fusedGyro(k,3)/Fs;            % integrate z-rate
  yaw_cf(k)= alpha*(yaw_cf(k-1)+dy) ...
             + (1-alpha)*yaw(k);      % blend in mag heading
end





%% Compare Vertical Acceleration vs Depth
% 1D complimentary filter
% initialize
z_cf  = depth(1);    % start from first measurement
vz_cf = 0;           % assume zero v at start
alpha = 0.98;        % trust in accel vs depth
az_lin = zeros(N,1);
for k = 2:N
  % 1) get vertical accel in nav frame (compensate tilt)
  az_nav = ax(k)*sin(pitch(k)) ...
         + ay(k)*(-sin(roll(k))*cos(pitch(k))) ...
         + az(k)* (cos(roll(k))*cos(pitch(k)));
  az_lin(k) = az(k) - 9.80665;

  % 2) predict
  vz_pred = vz_cf + az_lin(k)*dt;
  z_pred  = z_cf  + vz_cf*dt;

  % 3) correct toward depth measurement
  dz       = depth(k) - z_pred;
  z_cf     = z_pred  + (1-alpha)*dz;
  vz_cf    = vz_pred +  alpha*(dz/dt);

  zEst(k)  = z_cf;
  vzEst(k) = vz_cf;
end
z_cf = zEst;
vz_cf = vzEst;

% Bias estimation and dead reckoning
% Add bias state b, and use a small learning rate γ
b = 0;           % initial z-bias
y = 0.01;        % bias learning gain

vz = 0;
z_int = 0;
for k = 2:N
  % compensated accel
  a_zc  = az_lin(k) - b;  
  vz    = vz + a_zc*dt;
  z_int = z_int + vz*dt;

  % depth error
  e = depth(k) - z_int;

  % bias learning (drives e→0)
  b = b + y * e;

  % optionally reset z_int to depth when |e| small
  if abs(e) < 0.05
     z_int = depth(k);
     vz    = 0;
  end

  zEst(k) = z_int;
  biasLog(k) = b;
end
z_ab = zEst;
bias_ab = biasLog;

%% 3-state kalman
% state: [ z; vz; b ]    (3×1)
x   = [depth(1); 0; 0];
P   = diag([0.1, 0.1, 0.01]);
Q   = diag([0.001, 0.01, 1e-7]);  % tune if needed
Rz  = (0.05)^2;                   % depth noise variance
% Pre-allocate
x_fwd   = zeros(3, N);
P_fwd   = zeros(3, 3, N);
x_pred  = zeros(3, N);
P_pred  = zeros(3, 3, N);

for k = 2:N
  % 1) predict with accel
  A     = [1 dt -dt; 0 1  0;  0 0 1];
  B     = [dt; 0; 0];
  u     = az_lin(k);                % from §1
  x_p   = A*x + B*u;
  P_p   = A*P*A' + Q;

  % store prediction
  x_pred(:,k) = x_p;
  P_pred(:,:,k) = P_p;

  dz      = depth(k) - depth(k-1);
  vz_meas = dz / dt;                % pseudo-measurement of v_z

  % build 2×1 measurement vector
  z_meas2 = [ depth(k);  vz_meas ];
  
  % 2×3 measurement matrix: observe z and v_z
  H2 = [ 1 0 0
         0 1 0 ];
  % 2×2 measurement noise: depth noise + velocity-from-depth noise
  R2 = diag([ Rz,  (Rz/dt^2) ]);
  
  % Kalman correction with 2-dim measurement
   % --- change to ---
  K   = P_p * H2' / (H2 * P_p * H2' + R2);      % use P_p, not P
  x   = x_p     + K * (z_meas2 - H2*x_p);      % use x_p (3×1)
  P   = (eye(3) - K*H2) * P_p;                 % and P_p
  % store filtered
  x_fwd(:,k) = x;
  P_fwd(:,:,k) = P;

  % log
  zEst(k)  = x(1);
  vzEst(k) = x(2);
  bEst(k)  = x(3);
end
z_kf = zEst;
vz_kf = vzEst;
b_kf = bEst;

%% RTS Smoother on Kalman Filter
% Pre-allocate smoothed arrays
x_s = zeros(3,N);
P_s = zeros(3,3,N);

% Initialize with final filtered state
x_s(:,N) = x_fwd(:,N);
P_s(:,:,N) = P_fwd(:,:,N);

for k = N-1 : -1 : 1
  % same A as before
  A     = [1 dt -0.5*dt^2; 0 1  -dt;  0 0 1];
  % smoother gain
  G = P_fwd(:,:,k) * A' / P_pred(:,:,k+1);

  % state smoothing
  x_s(:,k) = x_fwd(:,k) + G*( x_s(:,k+1) - x_pred(:,k+1) );

  % covariance smoothing (optional if you only need states)
  P_s(:,:,k) = P_fwd(:,:,k) + ...
       G*( P_s(:,:,k+1) - P_pred(:,:,k+1) )*G';
end
z_smooth   = x_s(1,:);
vz_smooth  = x_s(2,:);
b_smooth   = x_s(3,:);

figure('Name','Depth Tracking');
hold on;
plot(t, depth,  'b-', 'DisplayName','Measured z');
plot(t, z_kf,   'r-','DisplayName','Kalman filter');
plot(t, z_smooth, 'g-', 'DisplayName', 'RTS Smoothed')
plot(t, depth_des, 'k-', 'DisplayName', 'Desired Depth')
grid on;
xlabel('time (s)'); ylabel('z (m)');
legend('Location','best');
title('Depth Estimates vs. Measured Depth');

%% Error Plot
figure('Name', 'Filter Error')
n = size(z_kf);
zkf_err = zeros(n);
zsmth_err = zeros(n);
zkf_errp = zeros(n);
zsmth_errp = zeros(n);
for k = 1:n(2)
    zkf_err(k) = z_kf(k) - depth(k);
    zsmth_err(k) = z_smooth(k) - depth(k);
    zkf_errp(k) = 100*(z_kf(k) - depth(k))./depth(k);
    zsmth_errp(k) = 100*(z_smooth(k) - depth(k))./depth(k);
end

subplot(2,1,1);
plot(t, zkf_err, 'r-', 'DisplayName','Kalman error [m]'); hold on;
plot(t, zsmth_err, 'g-', 'DisplayName','RTS error [m]');
grid on;
xlabel('time (s)'); ylabel('error (m)');
legend('Location','best');
title('Depth Tracking Error (m)');
ylim([-0.1 0.1]);
subplot(2,1,2);
plot(t, zkf_errp, 'r-', 'DisplayName','Kalman error [m]'); hold on;
plot(t, zsmth_errp, 'g-', 'DisplayName','RTS error [m]');
xlabel('time (s)'); ylabel('error (%)');
legend('Location', 'Best');
title('Depth Tracking Error (%)')
grid on;
ylim([-10 10]);

% Calculate trimmed mean error and display
ekf_trimmed_mean_error = trimmean(abs(zkf_errp),5);
rts_trimmed_mean_error = trimmean(abs(zsmth_errp),5);
fprintf('Trimmed Mean Absolute percent error for EKF Depth: %.2f%%\n', ekf_trimmed_mean_error);
fprintf('Trimmed Mean Absolute percent error for RTS Depth: %.2f%%\n', rts_trimmed_mean_error);
fprintf('\n');
% Calculate noise of depth signals
depth_smooth = smoothdata(depth, 'movmean', 10);
depth_noise = depth - depth_smooth;
depth_noise_mad = mad(depth_noise);

z_ekf_smooth = smoothdata(z_kf, 'movmean', 10);
z_ekf_noise = z_kf - z_ekf_smooth;
z_ekf_noise_mad = mad(z_ekf_noise);

z_rts_smooth = smoothdata(z_smooth, 'movmean', 10);
z_rts_noise = z_smooth - z_rts_smooth;
rts_noise_mad = mad(z_rts_noise);

% znoise_ekf_norm   = z_ekf_norm   - smoothdata(z_ekf_norm, 'movmean', 10);
% znoise_deriv_norm = depth_deriv_norm - smoothdata(depth_deriv_norm, 'movmean', 10);
% znoise_rts_norm = z_rts_norm - smoothdata(z_rts_norm, 'movmean', 10);
% 
% mad_ekfz_norm   = mad(znoise_ekf_norm);
% mad_z_norm = mad(znoise_deriv_norm);
% mad_rtsz_norm   = mad(znoise_rts_norm);

fprintf(['Normalized EKF Z Position noise MAD: %.4f\n' ...
    'Normalized RTS Z Position noise MAD: %.4f\n' ...
    'Normalized Derivative Z Position noise MAD: %.4f\n'], ...
    z_ekf_noise_mad, rts_noise_mad, depth_noise_mad);

fprintf('\n')


%% Vertical Velocity 
dxdt = gradient(depth,dt);
intAccel = cumtrapz(az_lin);
figure('Name','Vertical Velocity');
%plot(t, vz_cf,  'r--','DisplayName','comp. filter');
subplot(2,1,1);
hold on;
plot(t, vz_kf,  'k-','DisplayName','Kalman Filter');
plot(t, vz_smooth, 'b-', 'DisplayName', 'RTS Smoothed');
%plot(t, depth, 'r', 'DisplayName', 'Depth');
grid on;
xlabel('time (s)'); ylabel('v_z (m/s)');
legend('Location','best');
title('Estimated Vertical Velocity From Kalman Filter + RTS');
subplot(2,1,2);
plot(t, dxdt, 'p-', 'DisplayName', 'Estimate from Depth'); hold on
plot(t, intAccel, 'r-', 'DisplayName', 'Integral of Accelerometer Z');
grid on;
xlabel('time (s)'); ylabel('v_z (m/s)');
legend('Location','best');
title('Estimated Vertical Velocity From Position Derivative and Accelerometer Integration');


% estimate difference in variance between position derivative and ekf
vz_ekf_norm   = vz_kf   / rms(vz_kf);
vz_deriv_norm = dxdt / rms(dxdt);
vz_rts_norm = vz_smooth / rms(vz_smooth);

noise_ekf_norm   = vz_ekf_norm   - smoothdata(vz_ekf_norm, 'movmean', 10);
noise_deriv_norm = vz_deriv_norm - smoothdata(vz_deriv_norm, 'movmean', 10);
noise_rts_norm = vz_rts_norm - smoothdata(vz_rts_norm, 'movmean', 10);

mad_ekf_norm   = mad(noise_ekf_norm);
mad_deriv_norm = mad(noise_deriv_norm);
mad_rts_norm   = mad(noise_rts_norm);

fprintf(['Normalized EKF Z Velocity noise MAD: %.4f\n' ...
    'Normalized RTS Z Velocity noise MAD: %.4f\n' ...
    'Normalized Derivative Z Velocity noise MAD: %.4f\n'], ...
    mad_ekf_norm, mad_rts_norm, mad_deriv_norm);

figure('Name','Accel Bias Estimate');
%plot(t, bias_ab, 'g-.','DisplayName','adaptive bias');
hold on;
plot(t, b_kf,    'r-','DisplayName','KF bias','LineWidth',1.5);
grid on;
xlabel('time (s)'); ylabel('bias (m/s^2)');
legend('Location','best');
title('Estimated Z‐axis Accel Bias');


figure('Name','Fusion Dashboard','Position',[100 100 800 600]);

subplot(3,1,1);
plot(t, depth, 'b'); hold on;
plot(t, z_kf,  'k', 'LineWidth',1.1);
ylabel('z (m)'); legend('meas','KF'); grid on;

subplot(3,1,2);
plot(t, vz_kf,'k','LineWidth',1.1);
ylabel('v_z (m/s)'); grid on;

subplot(3,1,3);
plot(t, b_kf,'k','LineWidth',1.1);
ylabel('bias'); xlabel('time (s)'); grid on;

sgtitle('3‐State Kalman Filter Performance');

