clear
clf
source = 'firstPierRunNew.mat'; % Raw Data filename
Fs = 10.1;
dt = 1/Fs;

[acc, gyro, mag] = calibrateData1(source);
fusedAcc = FuseData.fuseAccel(acc, Fs);
fusedGyro = FuseData.fuseGyro(gyro);
R = load(source);
depth = R.z;
N = size(depth);

N = N(1);

ax = fusedAcc(:,1);
ay = fusedAcc(:,2);
az = fusedAcc(:,3);

az_lin = az - 9.80665;

%% 3-state kalman
% state: [ z; vz; b ]    (3×1)
x   = [depth(1); 0; 0];
P   = diag([0.1, 0.1, 0.01]);
Q   = diag([0.001, 0.01, 1e-6]);  % tune if needed
Rz  = (0.05)^2;                   % depth noise variance
Rv  = (0.105)^2;                   % velocity noise variance
% Pre-allocate
x_fwd   = zeros(3, N);
P_fwd   = zeros(3, 3, N);
x_pred  = zeros(3, N);
P_pred  = zeros(3, 3, N);
zEst = zeros(N,1);
vzEst = zeros(N,1);
bEst = zeros(N,1);
nis = zeros(N,1);
vz_meas = sgolayfilt(double(depth), 1, 9);
for k = 2:N
  % 1) predict with accel
  A     = [1 dt -0.5*dt^2; 0 1  -dt;  0 0 1];
  B     = [0.5*dt^2; dt; 0];
  u     = az_lin(k);                % from §1
  x_p   = A*x + B*u;
  P_p   = A*P*A' + Q;

  % store prediction
  x_pred(:,k) = x_p;
  P_pred(:,:,k) = P_p;

  dz      = depth(k) - depth(k-1);
  % vz_meas = dz / dt;                % pseudo-measurement of v_z
  
  % build 2×1 measurement vector
  z_meas2 = [ depth(k);  vz_meas(k) ];
  
  % 2×3 measurement matrix: observe z and v_z
  H2 = [ 1 0 0
         0 1 0 ];
  % 2×2 measurement noise: depth noise + velocity-from-depth noise
  R2 = diag([ Rz,  (Rv/dt^2) ]);
  
  
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

  innov = z_meas2 - H2*x_p;
  nis(k) = innov' / (H2*P_p*H2' + R2) * innov;
end
% plot(nis);

z_kf = zEst;
vz_kf = vzEst;
b_kf = bEst;

n = size(z_kf);
zkf_err = zeros(n);
zkf_errp = zeros(n);
for k = 1:n(1)
    zkf_err(k) = z_kf(k) - depth(k);
    zkf_errp(k) = 100*(z_kf(k) - depth(k))/depth(k);
end

ekf_trimmed_mean_error = trimmean(abs(zkf_errp),5);
fprintf('Trimmed Mean Absolute percent error for EKF Depth: %.2f%%\n', ekf_trimmed_mean_error);
fprintf('\n');
% Calculate noise of depth signals
depth_smooth = smoothdata(depth, 'movmean', 10);
depth_noise = depth - depth_smooth;
depth_noise_mad = mad(depth_noise);
z_ekf_smooth = smoothdata(z_kf, 'movmean', 10);
z_ekf_noise = z_kf - z_ekf_smooth;
z_ekf_noise_mad = mad(z_ekf_noise);
fprintf(['Normalized EKF Z Position noise MAD: %.4f\n' ...
    'Normalized Measured Z Position noise MAD: %.4f\n'], ...
    z_ekf_noise_mad, depth_noise_mad);

fprintf('\n')

% estimate difference in variance between position derivative and ekf
dxdt = gradient(depth,dt);
vz_ekf_norm   = vz_kf   / rms(vz_kf);
vz_deriv_norm = dxdt / rms(dxdt);
noise_ekf_norm   = vz_ekf_norm   - smoothdata(vz_ekf_norm, 'movmean', 10);
noise_deriv_norm = vz_deriv_norm - smoothdata(vz_deriv_norm, 'movmean', 10);
mad_ekf_norm   = mad(noise_ekf_norm);
mad_deriv_norm = mad(noise_deriv_norm);
fprintf(['Normalized EKF Z Velocity noise MAD: %.4f\n' ...
    'Normalized Derivative Z Velocity noise MAD: %.4f\n'], ...
    mad_ekf_norm, mad_deriv_norm);

ct = numel(nis(nis<=6));

fprintf('Ratio of NIS samples >= 6 = %0.3f \n', 1-ct/N);