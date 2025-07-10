function [roll, pitch, yaw] = computeOrientation(acc,gyro,mag)
ax = acc(:,1);
ay = acc(:,2);
az = acc(:,3);
gx = gyro(:,1);
gy = gyro(:,2);
gz = gyro(:,3);
mx = mag(:,1);
my = mag(:,2);
mz = mag(:,3);

dt = 1/10.1;

N = numel(ax);
roll  = zeros(N,1);
pitch = zeros(N,1);
yaw   = zeros(N,1);
alpha = 0.98;  % complementary filter weight

% tilt-compensated mag for initial yaw
% Mx2 =  mx(1)*cos(pitch(1)) + mz(1)*sin(pitch(1));
% My2 =  mx(1)*sin(roll(1))*sin(pitch(1)) + my(1)*cos(roll(1)) ...
%      - mz(1)*sin(roll(1))*cos(pitch(1));
yaw(1) = 0;
%yaw(1) = atan2(-My2, Mx2);

% --- main loop ---
for k = 2:N
  % 1) gyro integration
  r_gyro = roll(k-1)  + gx(k)*dt;
  p_gyro = pitch(k-1) + gy(k)*dt;
  y_gyro = yaw(k-1)   + gz(k)*dt;
  
  % 2) accel‐based angles
  r_acc = atan2( ay(k), az(k) );
  p_acc = atan2(-ax(k), sqrt(ay(k)^2 + az(k)^2));
  
  % 3) tilt-comp mag heading
  Mx2 =  mx(k)*cos(p_acc) + mz(k)*sin(p_acc);
  My2 =  mx(k)*sin(r_acc)*sin(p_acc) + my(k)*cos(r_acc) ...
       - mz(k)*sin(r_acc)*cos(p_acc);
  y_mag = atan2(-My2, Mx2);
  
  % 4) complementary filter
  roll(k)  = alpha*r_gyro  + (1-alpha)*r_acc;
  pitch(k) = alpha*p_gyro  + (1-alpha)*p_acc;
  yaw(k)   = alpha*y_gyro  + (1-alpha)*y_mag;
end

% convert to degrees
roll  = rad2deg(roll);
pitch = rad2deg(pitch);
yaw   = rad2deg(yaw);
end

