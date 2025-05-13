%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Maximum Path Search & Particle Filtering

% load variables
S = load('output/PMMmap.mat');
PMMmap = S.PMMmap;

% Take the maximum PMM score over all Rx channels
PMM_combined = max(PMMmap, [], 3);    % size = [R × F]
[R, F] = size(PMM_combined);

% Estimate per-range baseline N(r)
Nvec = mean(PMM_combined, 2);   % [R×1]

% Compute per-frame gain G(t)
Gvec = (Nvec' * PMM_combined) / (Nvec' * Nvec);  % [1×F]

% Subtract scaled baseline
Spp = PMM_combined - Nvec * Gvec;     % [R×F]

%% Dynamic-Programming 
% UAV kinematics → maximum jump in bins between frames

Td = (mmWave_device.chirp_ramp_time + mmWave_device.chirp_idle_time) ... 
            * 1e-6 * chirps_per_frame;          % time between frames (s)
Vmax  = mmWave_device.v_max;         % from your device struct (m/s)
Rres  = mmWave_device.range_res;     % (m/bin)
K     = ceil( Vmax * Td / Rres );    %

% allocate
theta = zeros(R, F);      % best cumulative score ending at (r,t)
prev  = zeros(R, F);      % back-pointers

% init with first frame
theta(:,1) = Spp(:,1);

% forward recursion
for t = 2:F
    for r = 1:R
        kmin = max(1,   r - K);
        kmax = min(R, r + K);
        [bestPrev, idx] = max( theta(kmin:kmax, t-1) );
        theta(r,t)     = bestPrev + Spp(r,t);
        prev(r,t)      = kmin + idx - 1;
    end
end

% back-track to get the raw DP path g_dp
g_dp = zeros(1, F);
[~, g_dp(F)] = max(theta(:,F));
for t = F:-1:2
  g_dp(t-1) = prev( g_dp(t), t );
end

% convert bin-indices → range (m)
range_bins = (0:R-1) * Rres;  
track_dp   = range_bins(g_dp);

%% 4. Particle-Filter smoothing
Np      = 5000;                 % number of particles
particles = zeros(2, Np);       % rows = [range_bin; velocity_bin]
weights   = ones(1, Np)/Np;     % uniform init

% process noise std (tune if needed)
σr = 2;    % bins
σv = 1;    % bins/sec

% initialize around DP path first point
particles(1,:) = g_dp(1) + σr*randn(1,Np);
particles(2,:) =   0  + σv*randn(1,Np);

% store estimates
est = zeros(2, F);
est(:,1) = [g_dp(1); 0];

for t = 2:F
  % 4.1 Predict
  particles(1,:) = particles(1,:) + particles(2,:)*Td + σr*randn(1,Np);
  particles(2,:) = particles(2,:)              + σv*randn(1,Np);
  
  % 4.2 Update weights by likelihood ∝ cleaned PMM score
  bin_idx = round( particles(1,:) );
  bin_idx = min(max(bin_idx,1), R);    % clamp into [1,R]
  likelihood = Spp( bin_idx, t ).';
  weights = weights .* likelihood;
  weights = weights + eps;              % avoid zeros
  weights = weights / sum(weights);
  
  % 4.3 Resample (multinomial)
  idx      = randsample(1:Np, Np, true, weights);
  particles = particles(:, idx);
  weights   = ones(1,Np)/Np;
  
  % 4.4 Estimate (weighted mean)
  est(:,t) = particles * weights.';
end

% convert particle-filter output to meters & m/s
track_pf_range = range_bins( round(est(1,:)) );
track_pf_vel   = (est(2,:) * Rres) / Td;

%% 5. Plot everything
t_axis = (0:F-1) * Td;
figure; 
subplot(2,1,1);
imagesc(t_axis, range_bins, Spp);
axis xy; hold on;
plot(t_axis, track_dp, '-w','LineWidth',2);
plot(t_axis, track_pf_range,'-r','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Range (m)');
legend('DP path','PF smoothed');
title('UAV Range Tracking');

subplot(2,1,2);
plot(t_axis, track_pf_vel);
xlabel('Time (s)'); ylabel('Radial speed (m/s)');
title('Estimated UAV Velocity');
