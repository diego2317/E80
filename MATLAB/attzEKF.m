function x = attzEKF(gyro, accel, mag, depth)
% gyro, accel, mag : N×3  (rad/s, m/s², µT)  already calibrated
% zMeas            : N×1  (m)  depth (NaN when unavailable)
% RETURNS          : struct with q( N×4 ), z, vz
Fs = 10.1; % Hz
g     = 9.80665;         % gravity
dt    = 1/Fs;            % constant step
N     = size(gyro,1);

%% 0.  σ-estimates from datasheets
sig_g = 0.0013089969*2; % rad/s
sig_a = 0.02941995*2; % m/s^2
sig_m = 3; % really high value, uTesla
sig_z = 0.02; % idfk, meters

sig_bg = 0.0002;                                 

%% 1.  EKF object ---------------------------------------------------------
% ––– Attitude EKF setup –––
% State: q (4) , bg (3)  → 7×1
x0_att = [1 0 0 0  0 0 0]';    % init quaternion & zero bias

ekf_att = extendedKalmanFilter(@fStateAtt, @hAccelAtt, x0_att, ...
    'HasAdditiveProcessNoise',     true, ...
    'HasAdditiveMeasurementNoise', true);

% covariances
ekf_att.ProcessNoise = 1e-6 * eye(7);
%ekf_att.ProcessNoise     = blkdiag( 0.25*sig_g^2*dt^2*eye(3),  sig_bg^2*dt*eye(3) );
ekf_att.MeasurementNoise = sig_a^2 * eye(3);    % accelerometer only

% ––– Depth EKF setup –––
% State: z , vz  → 2×1
x0_dep = [ depth(1); 0 ];  

ekf_dep = extendedKalmanFilter(@fStateDep, @hDepthDep, x0_dep);
ekf_dep.ProcessNoise = single(1e-4);
% ekf_dep.ProcessNoise     = [ (sig_a^2*dt^3)/3, 0 ; 0, sig_a^2*dt ];
ekf_dep.MeasurementNoise = single(sig_z^2);  % scalar

%% 2.  output logs --------------------------------------------------------
% Preallocate logs:
qLog  = zeros(N,4);
bgLog = zeros(N,3);
zLog  = zeros(N,1);
vzLog = zeros(N,1);

%% 3.  main loop ----------------------------------------------------------
for k = 1:N
  % predict with gyro
  predict(ekf_att, gyro(k,:)*dt);

  % correct with accel
  correct(ekf_att, accel(k,:));

  % optionally correct with mag
  if all(isfinite(mag(k,:)))
    ekf_att.MeasurementFcn   = @hMagAtt;
    ekf_att.MeasurementNoise = sig_m^2*eye(3);
    correct(ekf_att, mag(k,:));
    ekf_att.MeasurementFcn   = @hAccelAtt;  % restore
  end

  qLog(k,:) = ekf_att.State(1:4)';
  bgLog(k,:) = ekf_att.State(5:7)';

  % extract body→nav a_z
  qk    = quaternion(qLog(k,:));
  aNav  = quatrotate(qk, accel(k,:));
  azNav = aNav(3) - g;

  % predict with vertical accel
  predict(ekf_dep, azNav*dt);

  % correct with depth[k]
  if isfinite(depth(k))
    correct(ekf_dep, depth(k));
  end

  zLog(k)  = ekf_dep.State(1);
  vzLog(k) = ekf_dep.State(2);
end


x = [qLog zLog vzLog];
%save results.mat qLog zLog vzLog


end

%% -----------------------  local functions  -----------------------------
function xk1 = fStateAtt(xk, u)
  % xk = [q0 q1 q2 q3  bgx bgy bgz]'
  q  = quaternion(xk(1),xk(2),xk(3),xk(4));
  bg = xk(5:7);
  w  = u(:) - bg;         % bias‐corrected rate
  dq = quaternion(0, w(1), w(2), w(3));
  q  = normalize( q + 0.5*dq*q );
  xk1 = [ compact(q)'; bg ];
end

function z = hAccelAtt(xk)
  q = quaternion(xk(1),xk(2),xk(3),xk(4));
  z = quatrotate(q, [0 0 9.80665]);  % gravity direction in body
end

function z = hMagAtt(xk)
  q = quaternion(xk(1),xk(2),xk(3),xk(4));
  z = quatrotate(q, [1 0 0]);  % north unit vector
end

function xk1 = fStateDep(xk, u)
  % xk = [z; vz], u = azNav*dt
  z  = xk(1); vz = xk(2);
  vz1 = vz + u;       % integrate accel→velocity
  z1  = z  + vz*(1/10.1);   % integrate velocity→position
  xk1 = [z1; vz1];
end

function z = hDepthDep(xk)
  z = xk(1);          % direct measurement of z
end
