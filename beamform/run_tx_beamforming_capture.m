% Usage
% -----
%   >> run_tx_beamforming_capture( @chirpProfile_TxBF_USRR );
%
%   Replace the profile function handle with @chirpProfile_TxBF_SMRR or
%   @chirpProfile_TxBF_LRR for different range modes.
%---------------------------------------------------------------------
function run_tx_beamforming_capture(profileFcn)

    arguments
        profileFcn (1,1) function_handle % e.g. @chirpProfile_TxBF_USRR
    end

    %% 1.  House‑keeping --------------------------------------------------
    close all;
    clearvars -except profileFcn;

    % addpath(genpath('PostProc'));   % post‑processing utilities (if any)
    % addpath(genpath('RSTD'));       % mmWave Studio RTTT .NET wrappers

    %% 2.  User‑editable paths -------------------------------------------
    dllPath      = "C:\ti\mmwave_studio_02_01_00_00\mmWaveStudio\Clients\RtttNetClientController\RtttNetClientAPI.dll";
    psLutPath    = "./input/calibrConfig/phaseShifterCalibration.mat";
    mismatchPath = "./input/calibrConfig/phaseMismatchCalibration.mat";

    %% 3.  Load profile (chirp, frame, steering angles ...) --------------
    params = profileFcn();   % struct with all timing & geometry fields

    %% 4.  Connect to mmWave Studio RTTT API -----------------------------
    assert(init_rstd_conn(dllPath)==30000, ...
        "[RSTD] Connection failed – check DLL path or RadarStudio status.");

    %% 5.  Load calibration LUTs -----------------------------------------
    calib = load_calibration(psLutPath, mismatchPath, params.Tx_Ant_Arr_BF);

    %% 6.  Build per‑angle phase‑shifter tables --------------------------
    [Tx1_deg, Tx2_deg, Tx3_deg] = build_phase_luts(params, calib);

    %% 7.  Push configuration to each device -----------------------------
    activeDevs = find(params.RadarDevice);   % 1..4 for master+slaves

    for devId = activeDevs
        err = sensor_advance_frame_BF(devId, params, Tx1_deg, Tx2_deg, Tx3_deg);
        assert(err==30000, "[Dev %d] configuration failed (status %d)", devId, err);
    end

    fprintf("\n  All active AWR devices configured – ready to capture!\n");
end
