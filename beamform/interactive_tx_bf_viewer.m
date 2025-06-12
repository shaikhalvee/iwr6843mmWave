function interactive_tx_bf_viewer(data_folder)
% INTERACTIVE_TX_BF_VIEWER: Interactive viewer for TX beamforming post-processing results.
% Usage:
%   interactive_tx_bf_viewer('./output/txbf_in_drn_neg12_pos12_24db');
%
% data_folder: folder where rangeDopplerFFTmap.mat and *_params.mat are saved

    if nargin < 1
        data_folder = uigetdir(pwd, 'Select output folder containing the saved MAT files');
    end

    % Load data
    d = dir(fullfile(data_folder, '*rangeDopplerFFTmap.mat'));
    assert(~isempty(d), 'Cannot find rangeDopplerFFTmap.mat in the given folder');
    load(fullfile(data_folder, d(1).name), ...
        'all_to_plot', 'all_range_axis', 'all_doppler_axis', 'all_range_angle_stich');

    d2 = dir(fullfile(data_folder, '*_params.mat'));
    assert(~isempty(d2), 'Cannot find *_params.mat in the given folder');
    load(fullfile(data_folder, d2(1).name), 'params');

    nFrames = numel(all_to_plot); % params.Num_Frames
    nAngles = params.NumAnglesToSweep;
    anglesToSteer = params.anglesToSteer;

    % Precompute: check dimensions and fix if needed
    if nAngles == 1 && size(all_to_plot{1}, 4) == 1
        nAngles = 1;
    end

    % Figure
    hFig = figure('Name', 'TX Beamforming Interactive Viewer', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);
    
    % Sliders
    hFrame = uicontrol('Style', 'slider', 'Min', 1, 'Max', nFrames, ...
        'Value', 1, 'SliderStep', [1/(nFrames-1) 1/(nFrames-1)], ...
        'Position', [120 10 350 20]);
    hAngle = uicontrol('Style', 'slider', 'Min', 1, 'Max', nAngles, ...
        'Value', 1, 'SliderStep', [1/(nAngles-1) 1/(nAngles-1)], ...
        'Position', [600 10 350 20]);

    hFrameLabel = uicontrol('Style', 'text', 'Position', [30 10 80 20], ...
        'String', 'Frame: 1', 'HorizontalAlignment', 'left');
    hAngleLabel = uicontrol('Style', 'text', 'Position', [960 10 80 20], ...
        'String', sprintf('Angle: %d°', anglesToSteer(1)), 'HorizontalAlignment', 'left');

    % Axes handles for 4 plots
    hAx1 = subplot(2,2,1); % Range-Doppler Map
    hAx2 = subplot(2,2,2); % Range Profile
    hAx3 = subplot(2,2,3); % Doppler Profile
    hAx4 = subplot(2,2,4); % Range-Azimuth 3D

    % Callback function for updating plots
    function updatePlots(~,~)
        frameIdx = round(get(hFrame, 'Value'));
        angleIdx = round(get(hAngle, 'Value'));

        % Update labels
        set(hFrameLabel, 'String', sprintf('Frame: %d', frameIdx));
        set(hAngleLabel, 'String', sprintf('Angle: %d°', anglesToSteer(angleIdx)));

        to_plot = all_to_plot{frameIdx};
        range_axis = all_range_axis{frameIdx};
        doppler_axis = all_doppler_axis{frameIdx};
        range_angle_stich = all_range_angle_stich{frameIdx};

        % Defensive dimension handling
        if ndims(to_plot) == 4
            rd_map_per_angle = to_plot(:,:,:,angleIdx); % (R, D, Rx)
            rd_map_per_angle = mean(rd_map_per_angle, 3); % (R, D)
        elseif ndims(to_plot) == 3
            if size(to_plot,3) >= angleIdx
                rd_map_per_angle = to_plot(:,:,angleIdx);
            else
                rd_map_per_angle = to_plot(:,:,1);
            end
        elseif ndims(to_plot) == 2
            rd_map_per_angle = to_plot;
        else
            error('Unexpected to_plot dimensions.');
        end

        % RD map (dB)
        axes(hAx1); cla(hAx1);
        imagesc(doppler_axis, range_axis, 20*log10(abs(rd_map_per_angle)));
        axis xy;
        xlabel('Doppler (m/s)'); ylabel('Range (m)');
        title(sprintf('Range-Doppler Map, Angle = %d° (Frame %d)', ...
            anglesToSteer(angleIdx), frameIdx));
        colorbar;

        % Range Profile
        axes(hAx2); cla(hAx2);
        plot(range_axis, 20*log10(mean(abs(rd_map_per_angle),2)), 'LineWidth', 1.5);
        xlabel('Range (m)');
        ylabel('Power (dB)');
        title(sprintf('Range Profile, Frame %d', frameIdx));
        grid on;

        % Doppler Profile
        axes(hAx3); cla(hAx3);
        plot(doppler_axis, 20*log10(mean(abs(rd_map_per_angle),1)), 'LineWidth', 1.5);
        xlabel('Velocity (m/s)');
        ylabel('Power (dB)');
        title(sprintf('Doppler Profile, Frame %d', frameIdx));
        grid on;

        % Range-Azimuth (stich map) with 3D perspective (view(0,60))
        axes(hAx4); cla(hAx4);
        numAngles = nAngles;
        indices_1D = 1:numel(range_axis);

        sine_theta = sind(anglesToSteer);
        cos_theta = sqrt(1-sine_theta.^2);
        [R_mat, sine_theta_mat] = meshgrid(range_axis(indices_1D), sine_theta);
        [~, cos_theta_mat] = meshgrid(range_axis(indices_1D), cos_theta);
        x_axis = R_mat.*cos_theta_mat;
        y_axis = R_mat.*sine_theta_mat;

        % range_angle_stich: (range, angle) or (range, angle, ...)
        if ~ismatrix(range_angle_stich)
            range_angle_stich_2d = squeeze(range_angle_stich(:,:,1));
        else
            range_angle_stich_2d = range_angle_stich;
        end
        % Flip to match original display
        range_angle_stich_flipped = flipud(range_angle_stich_2d(indices_1D,:).');
        surf(y_axis, x_axis, abs(range_angle_stich_flipped).^0.2,'EdgeColor','none');
        view(0, 60);
        xlabel('meters')
        ylabel('meters')
        title('Stich range/azimuth (3D view)')
        colorbar;
        axis tight
    end

    % Set callbacks
    set(hFrame, 'Callback', @updatePlots);
    set(hAngle, 'Callback', @updatePlots);

    % Initial plot
    updatePlots();

end
