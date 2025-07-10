classdef FuseData
    methods(Static)
        function aFuse = fuseABoring(A)
            aFuse = (A{1} + A{3})/2; 
        end
        function aFuse = fuseAccel(A, Fs)
            fc_pre  = 3;              % pre-filter cut-off (Hz)
            fc_post = 2;              % post-filter cut-off (Hz)
            winSec  = 1.5;            % rolling-var window (s)
            tau     = 3;              % outlier gate (σ)

            [b1,a1] = butter(2, fc_pre /(Fs/2));      % 2nd-order LPF
            [b2,a2] = butter(1, fc_post/(Fs/2));      % 1st-order LPF
            Nwin    = max(3,round(Fs*winSec));

            % ---------- 1. pre-filter each sensor ------------------------------------
            M = numel(A);
            for k = 1:M
                A{k} = filtfilt(b1,a1, A{k});   % zero-phase
            end
            N = size(A{1},1);


            % ---------- 2. rolling variance per axis --------------------------------
            sig2 = zeros(N,3,M);
            for k = 1:M
                sig2(:,:,k) = movvar(A{k}, Nwin, 0, 'omitnan');
            end

            % ---------- 3. inverse-variance weights ---------------------------------
            w = 1 ./ max(sig2, 1e-6);          % avoid /0
            w = w ./ sum(w,3);                 % normalise  (Nx3xM)

            % ---------- 4. weighted mean & Hampel gate ------------------------------
            aFuse = zeros(N,3);
            for n = 1:N
                % preliminary fuse
                tmp = 0;
                for k = 1:M, tmp = tmp + w(n,:,k).*A{k}(n,:); end
                mu  = tmp;
            
                % Hampel: re-weight samples > τ·σ away
                for k = 1:M
                    dev = abs(A{k}(n,:) - mu);
                    mask = dev > tau*sqrt(sig2(n,:,k));
                    w(n,mask,k) = 0;           % drop that axis of that sensor
                end
                for j = 1:3      % for each axis
                    sw = sum(w(n,j,:));
                    if sw == 0
                        w(n,j,:) = 1/M;          % equal share among M sensors
                    else
                        w(n,j,:) = w(n,j,:)/sw;  % normalise as before
                    end
                end
                % final fuse
                for k = 1:M, aFuse(n,:) = aFuse(n,:) + w(n,:,k).*A{k}(n,:); end
            end
            
            % ---------- 5. post-filter ----------------------------------------------
            for k = 1:M
                bad = ~isfinite(A{k});
                if any(bad,'all')
                    A{k} = fillmissing(A{k},'linear','EndValues','nearest');
                end
                A{k} = filtfilt(b1,a1, A{k});        % zero-phase LPF
            end

        end

        function gFuse = fuseGyro(G)
            gFuse = (G{1} + G{2})/2;           % simple MVUE
        end
    end
end

