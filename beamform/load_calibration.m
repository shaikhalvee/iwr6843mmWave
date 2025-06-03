%-----------------------------------------------------------------------
function calib = load_calibration(psPath, mismatchPath, txSel)
% Loads phase‑shifter & mismatch LUTs and trims them to selected TX IDs.

    %% Phase‑shifter (PS) calibration – 63 steps × 12 channels
    S  = load(psPath, "Ph");
    ps = squeeze(S.Ph(:,1,:)).';       % → [12 × 63]
    ps(ps>0) = ps(ps>0) - 360;         % map to (‑360,0] range

    calib.psActual = ps(txSel, :);     % keep only used TXs

    %% Per‑channel mismatch LUT (optional)
    if exist(mismatchPath, "file")
        M = load(mismatchPath, "calibResult");
        calib.txMismatch = M.calibResult.TxMismatch;   % [3 × 4]
    else
        calib.txMismatch = zeros(3,4);
        warning("TX mismatch file not found – proceeding with zeros.");
    end
end
