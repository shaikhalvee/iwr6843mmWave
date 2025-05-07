function P = spectrumFolding(dopSpec, jMin, jMax)
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

  N = numel(dopSpec);
  best = 0;
  for j = jMin:jMax
    L = floor(N/j);
    % only keep a multiple of j
    X = dopSpec(1:(j*L));
    % reshape: each row sums one phase-aligned comb
    M = reshape(X, j, L);
    s = sum(M,2);
    best = max(best, max(s));
  end
  P = best;
end
