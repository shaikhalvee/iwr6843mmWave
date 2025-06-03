%-----------------------------------------------------------------------
function [idealDeg, psTx, psLut] = beamSteerPhaseCalc(bAng, psActual, dTx, d)
% Vectorised rewrite of TI's helper (no odd/even loops, no "find").
% Returns:
%   idealDeg – ideal continuous phase for each TX (nAng × nTx)
%   psTx     – programmed phase (deg) after quantisation (nTx × nAng)
%   psLut    – actual achieved phase from LUT (nTx × nAng)

    nTx   = size(psActual,1);
    nAng  = numel(bAng);
    LSB   = 5.625;                              % deg
    psAllowed = 0:LSB:360-LSB;                  % 64 values

    % Ideal continuous phase (deg) ------------------------------
    sinTerm   = sind(bAng.' * 2*d);             % (nAng × 1)
    idealDeg  = wrapTo360( sinTerm .* dTx );    % broadcasting → (nAng × nTx)

    % Build LUT look‑up matrix ----------------------------------
    lut = [zeros(nTx,1) -psActual];             % prepend 0, negate sign

    % For each TX, use interp1 to pick nearest LUT value ≥ / ≤ ideal
    psTx  = zeros(nTx, nAng);
    psLut = zeros(nTx, nAng);

    for tx = 1:nTx
        % Vectorised distance between ideal and LUT for this TX
        delta = lut(tx,:) - idealDeg(:,tx);     % (nAng × 64)

        % Odd TX → choose min positive delta (≥0). Even TX → max negative.
        if mod(tx,2) % odd index
            posIdx = delta >= 0;
            [~,col] = min(delta .* posIdx + ~posIdx*Inf, [], 2);
        else         % even index
            negIdx = delta <= 0;
            [~,col] = max(delta .* negIdx - ~negIdx*Inf, [], 2);
        end
        psTx(tx,:)  = psAllowed(col).';
        psLut(tx,:) = lut(tx,col).';
    end
end
