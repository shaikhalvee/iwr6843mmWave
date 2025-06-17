function interactive_tx_bf_viewer_paged()
% INTERACTIVE_TX_BF_VIEWER_PAGED: Paged viewer for huge per-frame TXBF results.

    frames_per_batch = 50;
    data_folder = './output/txbf_prk_drn_9_1_9/rangeDopplerFFTmap';
    config_folder = './output/txbf_prk_drn_9_1_9/';

    % Get all frame file names
    frame_files = dir(fullfile(data_folder, 'frame_*.mat'));
    total_frames = numel(frame_files);

    config_data = load(fullfile(config_folder, 'config.mat'));
    all_range_axis = config_data.all_range_axis;       % [N_range, N_frames]
    all_doppler_axis = config_data.all_doppler_axis;   % [N_doppler, N_frames]
    all_range_angle_stich = config_data.all_range_angle_stich;  % []

    % Load config (angles, params)
    d2 = dir(fullfile(config_folder, '*_params.mat'));
    assert(~isempty(d2), 'Cannot find *_params.mat in the given folder');
    load(fullfile(config_folder, d2(1).name), 'params');
    anglesToSteer = params.anglesToSteer;
    nAngles = params.NumAnglesToSweep;

    % Set up batching state
    curr_batch_start = 1;
    curr_batch_end = min(frames_per_batch, total_frames);

    % Load initial batch
    batch_data = load_batch(curr_batch_start, curr_batch_end, frame_files);

    % UI setup
    hFig = figure('Name', 'TX Beamforming Interactive Viewer (Paged)', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);

    hNext = uicontrol('Style', 'pushbutton', 'String', 'Next', ...
        'Position', [520 10 80 25], 'Callback', @next_batch);
    hPrev = uicontrol('Style', 'pushbutton', 'String', 'Previous', ...
        'Position', [20 10 80 25], 'Callback', @prev_batch);

    batch_frames = curr_batch_end - curr_batch_start + 1;
    hFrame = uicontrol('Style', 'slider', 'Min', 1, 'Max', batch_frames, ...
        'Value', 1, 'SliderStep', [1/(batch_frames-1 + eps) 1/(batch_frames-1 + eps)], ...
        'Position', [120 10 350 20]);
    hAngle = uicontrol('Style', 'slider', 'Min', 1, 'Max', nAngles, ...
        'Value', 1, 'SliderStep', [1/(nAngles-1 + eps) 1/(nAngles-1 + eps)], ...
        'Position', [600 10 350 20]);

    hFrameLabel = uicontrol('Style', 'text', 'Position', [430 10 80 20], ...
        'String', sprintf('Frame: %d/%d', curr_batch_start, total_frames), 'HorizontalAlignment', 'left');
    hAngleLabel = uicontrol('Style', 'text', 'Position', [960 10 80 20], ...
        'String', sprintf('Angle: %d°', anglesToSteer(1)), 'HorizontalAlignment', 'left');

    hAx1 = subplot(2,2,1); % Range-Doppler Map
    hCB1 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', 'Position', [160 370 80 20], 'Value', 1, 'Parent', hFig);
    hAx2 = subplot(2,2,2); % Range Profile
    hCB2 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', 'Position', [690 370 80 20], 'Value', 1, 'Parent', hFig);
    hAx3 = subplot(2,2,3); % Doppler Profile
    hCB3 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', 'Position', [160 150 80 20], 'Value', 1, 'Parent', hFig);
    hAx4 = subplot(2,2,4); % Range-Azimuth 3D

    % Update plot function
    function updatePlots(~,~)
        frameIdx = round(get(hFrame, 'Value'));
        angleIdx = round(get(hAngle, 'Value'));
        isLog1 = get(hCB1, 'Value');
        isLog2 = get(hCB2, 'Value');
        isLog3 = get(hCB3, 'Value');

        globalFrameIdx = curr_batch_start + frameIdx - 1;
        set(hFrameLabel, 'String', sprintf('Frame: %d/%d', globalFrameIdx, total_frames));
        set(hAngleLabel, 'String', sprintf('Angle: %d°', anglesToSteer(angleIdx)));

        D = batch_data{frameIdx};
        RD_map = abs(D.RD_map);
        to_plot = abs(mean(RD_map(:, :, :, angleIdx), 3));
        range_axis = all_range_axis{frameIdx};
        doppler_axis = all_doppler_axis{frameIdx};
        range_angle_stich = all_range_angle_stich{frameIdx};

        % Range-Doppler Map (axes hAx1)
        axes(hAx1); cla(hAx1);
        if isLog1
            rd_disp = 20*log10(to_plot + eps);
            imagesc(doppler_axis, range_axis, rd_disp);
            title('Range-Doppler Map (dB)');
            ylabel('Range (m)'); xlabel('Doppler (m/s)');
            colorbar;
        else
            imagesc(doppler_axis, range_axis, to_plot);
            title('Range-Doppler Map (linear)');
            ylabel('Range (m)'); xlabel('Doppler (m/s)');
            colorbar;
        end
        axis xy;

        % Range Profile (axes hAx2)
        axes(hAx2); cla(hAx2);
        if isLog2
            plot(range_axis, 20*log10(to_plot + eps), 'LineWidth', 1.0);
            title('Range Profile (dB)');
            ylabel('Power (dB)');
        else
            plot(range_axis, to_plot, 'LineWidth', 1.0);
            title('Range Profile (linear)');
            ylabel('Power (linear)');
        end
        xlabel('Range (m)');
        grid on;

        % Doppler Profile (axes hAx3)
        axes(hAx3); cla(hAx3);
        if isLog3
            plot(doppler_axis, 20*log10(to_plot' + eps), 'LineWidth', 1.0);
            title('Doppler Profile (dB)');
            ylabel('Power (dB)');
        else
            plot(doppler_axis, to_plot', 'LineWidth', 1.0);
            title('Doppler Profile (linear)');
            ylabel('Power (linear)');
        end
        xlabel('Velocity (m/s)');
        grid on;

        % Range-Azimuth (stub for now, add logic if you have range_angle_stich)
        axes(hAx4); cla(hAx4);
        % You can enhance this with your previous 3D plotting logic
        text(0.2,0.5,'Range-Azimuth plot here','FontSize',14);
    end

    % --- Paging logic ---
    function next_batch(~,~)
        if curr_batch_end < total_frames
            curr_batch_start = curr_batch_start + frames_per_batch;
            curr_batch_end = min(curr_batch_start + frames_per_batch - 1, total_frames);
            batch_data = load_batch(curr_batch_start, curr_batch_end, frame_files);
            set(hFrame, 'Min', 1, 'Max', curr_batch_end-curr_batch_start+1, 'Value', 1, ...
                'SliderStep', [1/max(1,(curr_batch_end-curr_batch_start)), 1/max(1,(curr_batch_end-curr_batch_start))]);
            updatePlots();
        end
    end

    function prev_batch(~,~)
        if curr_batch_start > 1
            curr_batch_start = max(1, curr_batch_start - frames_per_batch);
            curr_batch_end = curr_batch_start + frames_per_batch - 1;
            if curr_batch_end > total_frames
                curr_batch_end = total_frames;
            end
            batch_data = load_batch(curr_batch_start, curr_batch_end, frame_files);
            set(hFrame, 'Min', 1, 'Max', curr_batch_end-curr_batch_start+1, 'Value', 1, ...
                'SliderStep', [1/max(1,(curr_batch_end-curr_batch_start)), 1/max(1,(curr_batch_end-curr_batch_start))]);
            updatePlots();
        end
    end

    % Set callbacks
    set(hFrame, 'Callback', @updatePlots);
    set(hAngle, 'Callback', @updatePlots);
    set(hCB1, 'Callback', @updatePlots);
    set(hCB2, 'Callback', @updatePlots);
    set(hCB3, 'Callback', @updatePlots);

    % Initial plot
    updatePlots();

end

function batch_data = load_batch(idx_start, idx_end, all_files)
    n = idx_end - idx_start + 1;
    batch_data = cell(1,n);
    for i = 1:n
        frame_idx = idx_start + i - 1;
        batch_data{i} = load(fullfile(all_files(frame_idx).folder, all_files(frame_idx).name));
    end
end
