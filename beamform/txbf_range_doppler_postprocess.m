%----------------------------------------------------------------------
% TX Beamforming Range-Doppler Processing (single angle, long-range)
%----------------------------------------------------------------------
close all; clc; clearvars;

PMM_MAP_SAVE = 1;

% ----------------- USER CONFIG ---------------------------------------
adc_data_folder = 'D:\Documents\Drone_Data\data\txbf_in_ng15_ps15_30db';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder =  ['./output/' testRootFolder];
oldParamsFile = [output_folder filesep testRootFolder '_params.mat'];
calib_file      = './input/calibrConfig/calibrateResults_dummy.mat';

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

% all_RD_map = {};        % cell array for all frames (if #frames can differ per file)
all_range_axis = {};
all_doppler_axis = {};
all_range_angle_stich = {};
all_to_plot = {};

for i_file = 1:numel(fileIdx_unique)
    [fileNameStruct] = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 2:numValidFrames
        params.frameId = frameId;

        % ----------------- LOAD RAW ADC DATA (ALL RX, 1 ANGLE) ----------
        radar_data_txbf = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, params);
        radar_data_txbf = radar_data_txbf(:,:,params.TI_Cascade_RX_ID,:);

        % --------------- RANGE-DOPPLER PROCESSING & PLOT ---------------
        [RD_map, range_axis, doppler_axis, range_angle_stich] = calc_range_doppler_bmfrm(radar_data_txbf, params, BF_MIMO_ref);

        % Store for saving later
        % all_RD_map{end+1} = RD_map;
        all_range_axis{end+1} = range_axis;
        all_doppler_axis{end+1} = doppler_axis;
        all_range_angle_stich{end+1} = range_angle_stich;

        % ------- Plot: select RX channel or average ---------------------
        % If RD_map is 3D (range x doppler x RX), take mean across Rx
        % if it's 4D, (R, D, Rx, numAng)
        
        if ndims(RD_map) == 3
            % to_plot = abs(RD_map(:,:,1));     % visualize RX#1 only
            to_plot = mean(abs(RD_map), 3); % or average over RXs
        elseif ndims(RD_map) == 4
            to_plot = mean(mean(abs(RD_map),3), 4); % or average over RXs and numAng
        else
            to_plot = abs(RD_map);
        end
        % saving plot data
        all_to_plot{end+1} = to_plot;

        % --------------- DISPLAY ---------------------------------

        display_graph(params, to_plot, range_axis, doppler_axis, range_angle_stich, 30);
    end
end

% Save all processed RD maps *after* the loop
% saving only to_plot, RD_map is taking too much space
save(fullfile(output_folder, "rangeDopplerFFTmap.mat"), "all_to_plot", ...
    "all_range_axis", "all_doppler_axis", "all_range_angle_stich", '-v7.3');

save(fullfile(output_folder, [testRootFolder '_params.mat']), 'params', '-v7.3');


function display_graph(params, to_plot, range_axis, doppler_axis, range_angle_stich, startRangeIdx)
    frameId = params.frameId;
    logged_plot = 20*log10(to_plot);
    figure(1);
    set(1, 'Name', ['Frame ID:' num2str(frameId)]);
    
    subplot(2,2,1);
    imagesc(doppler_axis, range_axis(startRangeIdx:end), to_plot(startRangeIdx:end,:));
    axis xy; xlabel('Doppler (m/s)'); ylabel('Range (m)');
    title(sprintf('Range-Doppler map, Frame %d', frameId));
    colorbar; drawnow;

    % --------------- RANGE PROFILE (new code) ----------------

    subplot(2,2,2)
    plot(range_axis(startRangeIdx:end), logged_plot(startRangeIdx:end,:), 'LineWidth', 1.5);
    xlabel('Range (m)');
    ylabel('Power (dB)');
    title(sprintf('Range Profile, Frame %d', frameId));
    grid on;

    % --------------- 3D PLOT FOR ANGLE SWEEP ---------------------
    numAngles = params.NumAnglesToSweep;
    anglesToSteer = params.anglesToSteer;
    indices_1D = (startRangeIdx: params.Samples_per_Chirp) ;
    if (numAngles) > 1
        sine_theta = sind(anglesToSteer);
        cos_theta = sqrt(1-sine_theta.^2);
        [R_mat, sine_theta_mat] = meshgrid(indices_1D*params.rangeBinSize, sine_theta);
        [~, cos_theta_mat] = meshgrid(indices_1D,cos_theta);
        x_axis = R_mat.*cos_theta_mat;
        y_axis = R_mat.*sine_theta_mat;

        range_angle_stich = (range_angle_stich(indices_1D,:).');
        subplot(2,2,3);surf(y_axis, x_axis, abs(range_angle_stich).^0.2,'EdgeColor','none');
        %xlim([-5 5])
        %ylim([0 10]);
        view(2);
        xlabel('meters')
        ylabel('meters')
        title('stich range/azimuth')
    
        subplot(2,2,4);surf(y_axis, x_axis, abs(range_angle_stich).^0.2,'EdgeColor','none');
        %xlim([-5 5])
        %ylim([0 10]);
        view(0, 60);
        xlabel('meters')
        ylabel('meters')
        title('stich range/azimuth')
    end
    pause(0.1);
end