%% Start by loading constants like biases, etc
clear
cal = struct;
load IMU_calibration.mat
cal.acc(1).sens = S;
cal.acc(1).bias = bias;
cal.acc(1).R = R;
load IMU_calibration_1.mat
cal.acc(2).sens = S;
cal.acc(2).bias = bias;
cal.acc(2).R = R;
load IMU_calibration_2.mat
cal.acc(3).sens = S;
cal.acc(3).bias = bias;
cal.acc(3).R = R;

cal.gyro(1).bias = [0.037272, -0.057679, -0.011549];
cal.gyro(1).sens = [0.9652, 0.9750, 1.029];
cal.gyro(1).R = [1, -0.121, -0.0076;
                -0.0048, 1, -0.0066;
                -0.0509, -0.0766, 1];

cal.gyro(2).bias = [0.028394, -0.047741, -0.008058];
cal.gyro(2).sens = [0.9704, 0.9648, 1.0211];
cal.gyro(2).R = [1, -0.127, -0.0274;
                -0.0058, 1, -0.0293;
                -0.0536, -0.0507, 1];

cal.mag.offset = [7.3254, 32.9593, -32.8897];
cal.mag.msoft = [0.0228, -0.0010, -0.001;
                 0, 0.0240, 0.0014;
                 0, 0, 0.0237];

save('calibrationData.mat', 'cal');