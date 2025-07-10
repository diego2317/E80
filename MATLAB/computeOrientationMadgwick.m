%% computeOrientationMadgwick.m
% Usage:
%   [roll,pitch,yaw] = computeOrientationMadgwick(ax,ay,az,gx,gy,gz,mx,my,mz,dt,beta)
%
% Inputs:
%   ax,ay,az  – accel (m/s²)
%   gx,gy,gz  – gyro (rad/s)
%   mx,my,mz  – mag (µT)
%   dt        – sample period (s)
%   beta      – filter gain (≈0.1 works well)
%
% Outputs (degrees):
%   roll,pitch,yaw – Nx1

function [roll,pitch,yaw] = computeOrientationMadgwick(acc,gyro,mag)
    
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
    beta = 0.1;
    N = numel(ax);
    % pre-allocate
    roll  = zeros(N,1);
    pitch = zeros(N,1);
    yaw   = zeros(N,1);
    
    % initial quaternion (w, x, y, z)
    q = [1; 0; 0; 0];
    
    for k = 1:N
        q = MadgwickUpdate(q, gx(k), gy(k), gz(k), ax(k), ay(k), az(k), mx(k), my(k), mz(k), dt, beta);
        
        % convert to Euler (ZYX / yaw-pitch-roll)
        %   RPY: roll = φ, pitch = θ, yaw = ψ
        qw = q(1); qx = q(2); qy = q(3); qz = q(4);
        roll(k)  = atan2( 2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2) );
        pitch(k) = asin(  2*(qw*qy - qz*qx) );
        yaw(k)   = atan2( 2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2) );
    end
end

% —————————————————————————————————————————————————————
function q = MadgwickUpdate(q, gx, gy, gz, ax, ay, az, mx, my, mz, dt, beta)
    % Normalize measurements
    aNorm = sqrt(ax^2+ay^2+az^2);   ax = ax/aNorm; ay = ay/aNorm; az = az/aNorm;
    mNorm = sqrt(mx^2+my^2+mz^2);   mx = mx/mNorm; my = my/mNorm; mz = mz/mNorm;

    % quaternion components
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    % Reference direction of Earth’s magnetic field
    hx = 2*(mx*(0.5- qy^2 - qz^2) + my*( qx*qy - qw*qz) + mz*( qx*qz + qw*qy));
    hy = 2*(mx*( qx*qy + qw*qz) + my*(0.5- qx^2 - qz^2) + mz*( qy*qz - qw*qx));
    bx = sqrt(hx^2 + hy^2);
    bz = 2*(mx*( qx*qz - qw*qy) + my*( qy*qz + qw*qx) + mz*(0.5- qx^2 - qy^2));

    % Gradient descent corrective step
    F = [ 2*(qx*qz - qw*qy) - ax;
          2*(qw*qx + qy*qz) - ay;
          2*(0.5 - qx^2 - qy^2) - az;
          2*bx*(0.5 - qy^2 - qz^2) + 2*bz*( qx*qz - qw*qy) - mx;
          2*bx*( qx*qy - qw*qz) + 2*bz*( qw*qx + qy*qz) - my;
          2*bx*( qw*qy + qx*qz) + 2*bz*(0.5 - qx^2 - qy^2) - mz ];
      
    J = [ -2*qy,            2*qz,           -2*qw,            2*qx;
           2*qx,            2*qw,            2*qz,            2*qy;
           0,               -4*qx,           -4*qy,           0;
          -2*bz*qy,         2*bz*qz,        -4*bx*qy-2*bz*qw, -4*bx*qz+2*bz*qx;
          -2*bx*qz+2*bz*qx, 2*bx*qy+2*bz*qw, 2*bx*qx+2*bz*qz, -2*bx*qw+2*bz*qy;
           2*bx*qy,         2*bx*qz-4*bz*qx, 2*bx*qw-4*bz*qy,  2*bx*qx ];

    step = J'*F;
    step = step / norm(step);       % normalize step

    % Compute rate of change of quaternion
    qDot = 0.5 * [ -qx, -qy, -qz;
                    qw, -qz,  qy;
                    qz,  qw, -qx;
                   -qy,  qx,  qw ] * [gx;gy;gz]  - beta*step;

    % Integrate and renormalize
    q = q + qDot*dt;
    q = q / norm(q);
end
