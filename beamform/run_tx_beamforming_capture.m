% Usage
% -----
%   >> run_tx_beamforming_capture( @chirpProfile_TxBF_USRR );
%
%   Replace the profile function handle with @chirpProfile_TxBF_SMRR or
%   @chirpProfile_TxBF_LRR for different range modes.
%---------------------------------------------------------------------
function run_tx_beamforming_capture(angles, profileFcn, paths)

    arguments
        angles
        profileFcn (1,1) function_handle % e.g. @chirpProfile_TxBF_LRR
        paths
    end

    %% House‑keeping --------------------------------------------------
    % close all;
    % clearvars -except profileFcn angles paths;

    %% User‑editable paths -------------------------------------------
    paths.testRoot = ''; % the name of the test file/folder
    paths.outDir = '';
    paths.dllPath      = "C:\ti\mmwave_studio_02_01_01_00\mmWaveStudio\Clients\RtttNetClientController\RtttNetClientAPI.dll";
    paths.psLutPath    = "./input/calibrConfig/phaseShifterCalibration.mat";
    paths.mismatchPath = "./input/calibrConfig/phaseMismatchCalibration.mat";

    %% Load profile (chirp, frame, steering angles ...) --------------
    params = profileFcn(angles);   % struct with all timing & geometry fields

    %% Connect to mmWave Studio RTTT API -----------------------------
    % assert(init_rstd_conn(dllPath)==30000, ...
    %     "[RSTD] Connection failed – check DLL path or RadarStudio status.");

    %% Load calibration LUTs -----------------------------------------
    calib = load_calibration(paths.psLutPath, paths.mismatchPath, params.Tx_Ant_Arr_BF);

    %% Build per‑angle phase‑shifter tables --------------------------
    [Tx1_deg, Tx2_deg, Tx3_deg] = build_phase_luts(params, calib);

    %% Push configuration to each device -----------------------------
    activeDevs = find(params.RadarDevice);   % 1..4 for master+slaves

    for devId = activeDevs
        err = sensor_advance_frame_BF(devId, params, Tx1_deg, Tx2_deg, Tx3_deg);
        assert(err==30000, "[Dev %d] configuration failed (status %d)", devId, err);
    end

    fprintf("\n  All active AWR devices configured – ready to capture!\n");

    %% Save params
    capture_folder = ''; % folder to save the params of the testing
    save(fullfile(capture_folder, 'radar_params_capture.mat'), 'params');
    fprintf('[INFO] Saved radar params to %s\n', fullfile(capture_folder, 'radar_params_capture.mat'));
    
end
