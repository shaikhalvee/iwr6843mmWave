% ==========  PRE‑REQUISITES (already in your workspace)  =================
% RDcube_cplx   – complex Range–Doppler data, size [R × D × Rx × F]
% Rres,        range‑bin size (m)
% track_pf_range – PF‑smoothed range (m) for each frame (1×F)
% ------------------------------------------------------------------------

%% Load Data
S = load('output/fft_result_cube.mat');
P = load('output/UAVtracking.mat');
mmWave_device = S.mmWave_device;
RDcube_cplx = S.fft_complex_radar_cube;
Rres = mmWave_device.range_res;
track_pf_range = P.track_pf_range;

%% PARAMETERS -------------------------------------------------------------
lambda = mmWave_device.lambda;      % wavelength (m)
d = lambda/2;                       % element spacing (m)  (ISK‑ODS)
azGrid = -90:1:90;                  % scan grid (deg)
La = numel(azGrid);                 % # angle bins
jMin = 2;   
jMax = 20;                          % PMM fold search (same as before)
snapWin = 8;                        % Doppler snapshots for R̂
delta = 1e-3;                       % diagonal loading
% Td = (mmWave_device.chirp_ramp_time + mmWave_device.chirp_idle_time) *1e-6 * mmWave_device.num_chirp_per_frame;  % s
Td = mmWave_device.frame_periodicity / 1000;   % msec → seconds || time between frames. the better one

% Pre‑compute steering matrix  A (Rx × La)
k = 2*pi/lambda;
Rx = size(RDcube_cplx,3);
A  = exp( 1j * k * d * (0:Rx-1).' * sind(azGrid) );

%% 1.  Build the Angle–Time PMM (A‑PMM) matrix ----------------------------
F = size(RDcube_cplx,4);     
D = size(RDcube_cplx,2);
A_PMM = zeros(La, F);        % [angle × frame]

for f = 1:F
    % --- 1.1  pick UAV‑related range bin (closest integer)
    rBin = round(track_pf_range(f) / Rres) + 1;     % UAV‑range bin
    Xrd  = squeeze( RDcube_cplx(rBin, :, :, f) );   % [D × Rx]
    
    % --- 1.2  spatial covariance  (average small Doppler window)
    dMid      = round(D/2);
    dSel      = max(1,dMid-floor(snapWin/2)) : min(D,dMid+floor(snapWin/2));
    snapshots = Xrd(dSel, :).';                      % [Rx × Ns]
    Rcov      = (snapshots * snapshots.') / size(snapshots,2) + delta*eye(Rx);
    Ri        = inv(Rcov);                           % 4×4 here
    
    % --- 1.3  Capon weights for every angle (shared)
    denom = sum( (Ri*A) .* conj(A), 1 );             % 1×La
    W     = (Ri*A) ./ denom;                         % Rx × La   (each col: w(θ))
    
    % --- 1.4  Beamform: Y(D × La)  =  Xrd * conj(w)
    Y = Xrd * conj(W);                               % [D × La]
    
    % --- 1.5  PMM folding per angle
    for a = 1:La
        dopSpec = abs( Y(:,a) ).';                  % 1×D
        A_PMM(a,f) = spectrumFoldingPerDopSpec(dopSpec, jMin, jMax);
    end
end

%% 2.  Spectral subtraction over angles (same as range) -------------------
Nang = mean(A_PMM, 2);                               % eq.(12)
Gain = (Nang' * A_PMM) / (Nang' * Nang);             % 1×F   eq.(13)
Sang = A_PMM - Nang * Gain;                          % cleaned A‑PMM  eq.(14)

%% 3.  DP tracker in angle domain ----------------------------------------
%   Let max turn‑rate ≈ 30°/s → one‑frame jump ≤ J bins
angleRes = diff(azGrid(1:2));                % 1°
J = ceil( 30 * Td / angleRes );              % e.g. 3 bins

[La,~] = size(Sang);
thetaA = zeros(La,F);   prevA = zeros(La,F);
thetaA(:,1) = Sang(:,1);

for t = 2:F
  for a = 1:La
     jmin = max(1, a-J);  jmax = min(La, a+J);
     [best, idx] = max( thetaA(jmin:jmax,t-1) );
     thetaA(a,t) = best + Sang(a,t);
     prevA(a,t)  = jmin + idx - 1;
  end
end

aTrack = zeros(1,F);
[~,aTrack(F)] = max(thetaA(:,F));
for t = F:-1:2, aTrack(t-1) = prevA(aTrack(t),t); end
azimuthTrack = azGrid(aTrack);                       % degrees

%% 4.  OPTIONAL: particle‑filter smooth (same code as before) -------------
%  (replace range bins with angle bins; J becomes std, etc.)

%% 5.  Visualise -----------------------------------------------------------
timeAxis = (0:F-1)*Td;

figure;
subplot(2,1,1);
imagesc(timeAxis, azGrid, Sang);
axis xy; hold on;
plot(timeAxis, azimuthTrack,'-w','LineWidth',2);
xlabel('Time (s)'); ylabel('Azimuth (°)');
title('Angle–Time PMM with DP track');

subplot(2,1,2);
plot(timeAxis, azimuthTrack);
xlabel('Time (s)'); ylabel('Azimuth (°)');
title('Tracked UAV azimuth');


function P = spectrumFoldingPerDopSpec(dopSpec, jMin, jMax)
% dopSpec: 1×nDopplerFFT magnitude spectrum
% jMin/jMax: range of integer fold sizes to search
  n = length(dopSpec);
  Pj = zeros(1,jMax-jMin+1);
  for jj = jMin:jMax
    L = floor(n/jj);
    Dcut = dopSpec(1:jj*L);
    Dmat = reshape(Dcut, jj, L);
    % sum each row (align peaks), take the max row
    Pj(jj-jMin+1) = max( sum(Dmat,2) );
  end
  P = max(Pj);  % best folding score
end
