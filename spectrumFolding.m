% function P = spectrumFolding(dopSpec, jMin, jMax)
% P = spectrumFolding(dopSpec, jMin, jMax)
%   Search integer fold sizes j=jMin…jMax to align and sum
%   the doppler-comb peaks in |dopSpec|, returns the max score.
%
% Inputs:
%   dopSpec  – 1×N vector of magnitudes (one Doppler spectrum)
%   jMin,jMax  – smallest/largest integer “comb spacing” to try
%
% Output:
%   P  – best folding score (scalar)

%   N = numel(dopSpec);
%   best = 0;
%   for j = jMin:jMax
%     L = floor(N/j);
%     % only keep a multiple of j
%     X = dopSpec(1:(j*L));
%     % reshape: each row sums one phase-aligned comb
%     M = reshape(X, j, L);
%     s = sum(M,2);
%     best = max(best, max(s));
%   end
%   P = best;
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% New Style Coding
S = load('output/fft_result_cube.mat');
fft_2d_radar_cube = S.fft_2d_radar_cube;

%--- PARAMETERS ---------------------
jMin = 2;          % minimum fold size
jMax = 20;         % maximum fold size

[R, D, F, Rx] = size(fft_2d_radar_cube);

% pre-allocate: folding scores per range, frame, Rx
PMMmap = zeros(R, F, Rx);

for f = 1:F
    for rx = 1:Rx
        for r = 1:R
            % 1×D Doppler‐spectrum magnitude at (r,f,rx)
            dopSpec = abs( fft_2d_radar_cube(r, :, f, rx) );
            
            bestScore = 0;
            % search over integer fold‐sizes j
            for j = jMin:jMax
                M = floor(D / j);
                if M < 1
                    break;  % no valid folding for this j
                end
                
                % cut to an exact multiple of j, reshape into j×M
                Xcut = dopSpec(1:(j*M));
                Xmat = reshape(Xcut, j, M);
                
                % column‐wise average of aligned bins
                colAvg = sum(Xmat, 2) ./ M;  
                
                % peak alignment score for this j
                score_j = max(colAvg);
                
                bestScore = max(bestScore, score_j);
            end
            
            PMMmap(r, f, rx) = bestScore;
        end
    end
end

%--- OPTIONAL: combine across Rx channels -------------
% e.g. take maximum across Rx
PMM_combined = max(PMMmap, [], 3);   % size = [R × F]

% visualize the Range–Time PMM heat‐map
imagesc((1:F), (1:R), PMM_combined);
axis xy;
xlabel('Frame index');
ylabel('Range bin');
title('Range–Time PMM folding map');
colorbar;
