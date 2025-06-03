%-----------------------------------------------------------------------
function [Tx1_deg, Tx2_deg, Tx3_deg] = build_phase_luts(p, calib)
% Generates **hardware‑ready** phase tables (°) for each steering angle.

    % 1) Ideal → discrete PS lookup per selected TX
    [~, psTx, ~] = beamSteerPhaseCalc(p.anglesToSteer, ...
        calib.psActual, p.D_TX_BF, p.d_BF);            % → [numTx × nAng]

    % 2) Map back to 12‑slot cascade layout (zeros for unused TXs)
    ps12 = zeros(numel(p.anglesToSteer), 12);          % [nAng × 12]
    ps12(:, p.Tx_Ant_Arr_BF) = psTx.';                 % transpose!

    % 3) Reshape to device order (3 TX/ASIC × 4 ASICs)
    psDev = reshape(ps12.', 3, 4, []);                 % [3 × 4 × nAng]

    % 4) Add per‑device mismatch LUT, wrap to 0–360°
    psDev = wrapTo360(psDev + calib.txMismatch);

    % 5) Export as (nAng × 4) matrices per TX index
    Tx1_deg = squeeze(psDev(1,:,:)).';  % row = angle, col = dev
    Tx2_deg = squeeze(psDev(2,:,:)).';
    Tx3_deg = squeeze(psDev(3,:,:)).';
end
