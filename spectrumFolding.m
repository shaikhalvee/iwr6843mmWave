%% Spectrum Folding

clear all;

% load variables
S = load('output/fft_result_cube.mat');
fft_complex_radar_cube = S.fft_complex_radar_cube;

%--- PARAMETERS ---------------------
jMin = 2;          % minimum fold size
jMax = 20;         % maximum fold size

[numRangeBins, numDopplerBins, numFrames, numRx] = size(fft_complex_radar_cube);

% take normalized values
% mag = abs(fft_complex_radar_cube);
% noise_flr = median(mag(:));
% norm_fft = mag / noise_flr;

% pre-allocate: folding scores per range, frame, Rx
PMMmap = zeros(numRangeBins, numFrames, numRx);

for f = 1:numFrames
    for rx = 1:numRx
        for r = 1:numRangeBins
            % 1×D Doppler‐spectrum magnitude at (r,f,rx)
            % dopplerSpectrum = 20*log10(fliplr(abs(fft_complex_radar_cube(r, :, f, rx))));
            dopplerSpectrum = fliplr(abs(fft_complex_radar_cube(r, :, f, rx))); 
            % usually it was done before. but for the sake of input sanitization, we're performing the abs.
            
            bestScore = 0;
            % search over integer fold‐sizes j
            for j = jMin:jMax
                M = floor(numDopplerBins / j);
                if M < 1
                    break;  % no valid folding for this j
                end
                 
                % cut to an exact multiple of j, reshape into j×M
                Xcut = dopplerSpectrum(1:(j*M));
                Xmat = reshape(Xcut, j, M);
                
                % column‐wise average of aligned bins
                colAvg = sum(Xmat, 2) ./ M;  
                
                % peak alignment score for this j
                score_j = max(colAvg);
                
                bestScore = max(bestScore, score_j);
            end
            % PMM score saved for current range bin
            PMMmap(r, f, rx) = bestScore;
        end
    end
end

save('output/PMMmap', 'PMMmap');

%--- combine across Rx channels -------------
% e.g. take maximum across Rx
PMM_combined = max(PMMmap, [], 3);   % size = [R × F]

% visualize the Range–Time PMM heat‐map
imagesc((1:numFrames), (1:numRangeBins/4), PMM_combined(1:numRangeBins/4,:));
axis xy;
xlabel('Frame index');
ylabel('Range bin');
title('Range–Time PMM folding map');
colorbar;
