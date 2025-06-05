%----------------------------------------------------------------------
% TX Beamforming Range-Doppler Processing (single angle, long-range)
%----------------------------------------------------------------------
close all; clc; clearvars;

PMM_MAP_SAVE = 1;

% ----------------- USER CONFIG ---------------------------------------
adc_data_folder = 'D:\Documents\Drone_Data\data\tx_beamform_01\';
calib_file      = './input/calibrConfig/calibrateResults_high.mat';

% ----------------- LOAD PARAMS FROM JSON & CALIB ---------------------

paramsConfig = get_radar_config(adc_data_folder);

% paramsConfig = struct;
% I shouldn't repopulate here
paramsConfig.anglesToSteer = -23.45;
paramsConfig.NumAnglesToSweep = 1;
paramsConfig.Chirp_Frame_BF = 0;       % Only frame-based supported

% paramsConfig = parameter_gen_from_Jason(adc_data_folder, paramsConfig);

% Calibration (RX phase)
load(calib_file, 'calibResult');
BF_MIMO_ref = calibResult.RxMismatch;
% paramsConfig.Slope_MHzperus_calib = params.Slope_MHzperus;
% paramsConfig.TI_Cascade_RX_ID = paramsConfig.TI_Cascade_RX_ID; % (already set by parameter_gen)

% --------------- LOAD FILES & PROCESS EACH FRAME ----------------------
[fileIdx_unique] = getUniqueFileIdx(adc_data_folder);

for i_file = 1:numel(fileIdx_unique)
    [fileNameStruct] = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 2:numValidFrames
        paramsConfig.frameId = frameId;

        % ----------------- LOAD RAW ADC DATA (ALL RX, 1 ANGLE) ----------
        adc_cube = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, paramsConfig);
        adc_cube = adc_cube(:,:,paramsConfig.TI_Cascade_RX_ID,:);

        % --------------- RANGE-DOPPLER PROCESSING & PLOT ---------------
        [RD_map, range_axis, doppler_axis] = calc_range_doppler_bmfrm(adc_cube, paramsConfig, BF_MIMO_ref);

        save("rangeDopplerFFTmap.mat", "RD_map", '-v7.3');

        % --------------- DISPLAY (optionally replace with PMM) ---------
        % will work on to show PMM here
        figure(1); clf;
        imagesc(doppler_axis, range_axis, 20*log10(abs(RD_map)));
        axis xy; xlabel('Doppler (m/s)'); ylabel('Range (m)');
        title(sprintf('Range-Doppler map, Frame %d', frameId));
        colorbar; drawnow;
    end
end
