% logreader.m
% Use this script to read data from your micro SD card

clear;
%clf;

filenum = '012'; % file number for the data you want to read
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
% accelX = accelX .* 0.00980665;
% accelY = accelY .* 0.00980665;
% accelZ = accelZ .* 0.00980665;
% v0 = 0;
% x0 = 0;
% time = 1:length(accelX);
% dt = 0.1;
% time = time .* dt;
% sigma_a = 0.8825985;
% 
% velX = v0 + cumtrapz(time, accelX);
% velY = v0 + cumtrapz(time, accelY);
% velZ = v0 + cumtrapz(time, accelZ);
% 
% posX = x0 + cumtrapz(time, velX);
% posY = x0 + cumtrapz(time, velY);
% posZ = x0 + cumtrapz(time, velZ);
% 
% N = length(time);
% sigmaV = zeros(N,1);  % std dev of velocity at each time step
% sigmaP = zeros(N,1);  % std dev of position at each time step
% 
% % Assume at k=1 (the first sample) there's effectively no prior motion or uncertainty
% sigmaV(1) = 0;
% sigmaP(1) = 0;
% 
% % --- Step 3: Propagate uncertainties in a loop
% for k = 2:N
%     dt = time(k) - time(k-1);
% 
%     % Update velocity uncertainty
%     %   sigmaV(k)^2 = sigmaV(k-1)^2 + (dt^2)*sigma_a^2
%     sigmaV(k) = sqrt( sigmaV(k-1)^2 + (dt^2)*sigma_a^2 );
% 
%     % Update position uncertainty
%     %   sigmaP(k)^2 = sigmaP(k-1)^2 + (dt^2)*[sigmaV(k)^2]
%     sigmaP(k) = sqrt( sigmaP(k-1)^2 + (dt^2)*sigmaV(k)^2 );
% end
% 
% % --- Step 4: Create upper and lower 2-sigma bounds for plotting
% upperBound = posY + 2*sigmaP;   % +2 sigma
% lowerBound = posY - 2*sigmaP;   % -2 sigma
% 
% 
% figure(1);
% plot(posX, posY);
% 
% xlabel('X Position, meters');
% ylabel('Y Position, meters');
% title('X Position vs Y Position');
% 
% figure(2);
% hold on
% plot(time, posY, 'DisplayName','Position [m]');
% plot(time, upperBound, 'r--');
% plot(time, lowerBound, 'r--');
% xlabel('Time, seconds');
% ylabel('Y Position, meters');
% title('Y Position vs Time');
% legend('Y Position','Upper 95% Uncertainty Bound', 'Lower 95% Uncertainty Bound');
% 
% 
% hold off