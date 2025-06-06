%----------------------------------------------------------------------
% TX Beamforming Range-Doppler Processing (single angle, long-range)
%----------------------------------------------------------------------
close all; clc; clearvars;

PMM_MAP_SAVE = 1;

% ----------------- USER CONFIG ---------------------------------------
adc_data_folder = 'D:\Documents\Drone_Data\data\tx_bf_01';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder =  ['./output/' testRootFolder];
oldParamsFile = [output_folder filesep testRootFolder '_params.mat'];
calib_file      = './input/calibrConfig/calibrateResults_high.mat';

% ----------------- LOAD PARAMS FROM JSON & CALIB ---------------------
oldParams = load(oldParamsFile, 'params');
params = oldParams.params;

configFromAdcData = get_radar_config(params.anglesToSteer, [adc_data_folder '\']);
params.Slope_MHzperus = configFromAdcData.Slope_MHzperus;
params.TI_Cascade_RX_ID = configFromAdcData.TI_Cascade_RX_ID;
params.numRX = configFromAdcData.numRX;
params.D_RX = configFromAdcData.D_RX; 
params.calibrationInterp = configFromAdcData.calibrationInterp;
params.phaseCalibOnly = configFromAdcData.phaseCalibOnly;


% paramsConfig = struct;
% I shouldn't repopulate here
% paramsConfig.anglesToSteer = -23.45;
% paramsConfig.NumAnglesToSweep = 1;
% paramsConfig.Chirp_Frame_BF = 0;       % Only frame-based supported

% Calibration (RX phase)
load(calib_file, 'calibResult');
BF_MIMO_ref = calibResult.RxMismatch;
% paramsConfig.Slope_MHzperus_calib = params.Slope_MHzperus;
% paramsConfig.TI_Cascade_RX_ID = paramsConfig.TI_Cascade_RX_ID; % (already set by parameter_gen)

% --------------- LOAD FILES & PROCESS EACH FRAME ----------------------
[fileIdx_unique] = getUniqueFileIdx(adc_data_folder);

all_RD_map = {};        % cell array for all frames (if #frames can differ per file)
all_range_axis = {};
all_doppler_axis = {};

for i_file = 1:numel(fileIdx_unique)
    [fileNameStruct] = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 2:numValidFrames
        params.frameId = frameId;

        % ----------------- LOAD RAW ADC DATA (ALL RX, 1 ANGLE) ----------
        adc_cube = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, params);
        adc_cube = adc_cube(:,:,params.TI_Cascade_RX_ID,:);

        % --------------- RANGE-DOPPLER PROCESSING & PLOT ---------------
        [RD_map, range_axis, doppler_axis] = calc_range_doppler_bmfrm(adc_cube, params, BF_MIMO_ref);

        % Store for saving later
        all_RD_map{end+1} = RD_map;
        all_range_axis{end+1} = range_axis;
        all_doppler_axis{end+1} = doppler_axis;

        % ------- Plot: select RX channel or average (fix imagesc error!) ---
        % If RD_map is 3D (range x doppler x RX), take e.g. channel 1 or mean
        if ndims(RD_map) == 3
            to_plot = abs(RD_map(:,:,1));     % visualize RX#1 only
            % to_plot = mean(abs(RD_map),3); % or average over RXs
        else
            to_plot = abs(RD_map);
        end

        % --------------- DISPLAY (optionally replace with PMM) ---------
        % will work on to show PMM here
        figure(1); clf;
        imagesc(doppler_axis, range_axis, 20*log10(abs(to_plot)));
        axis xy; xlabel('Doppler (m/s)'); ylabel('Range (m)');
        title(sprintf('Range-Doppler map, Frame %d', frameId));
        colorbar; drawnow;
    end
end

% Save all processed RD maps *after* the loop
save(fullfile(output_folder, "rangeDopplerFFTmap.mat"), "all_RD_map", ...
    "all_range_axis", "all_doppler_axis", '-v7.3');

save(fullfile(output_folder, [testRootFolder '_params.mat']), 'params', '-v7.3');
