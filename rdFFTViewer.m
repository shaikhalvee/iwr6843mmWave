% get single chip adc data from /output. 
% visualize per Rx per Tx data

function rdFFTViewer()
    % Load the precomputed RD FFT cube
    S = load('output/fft_result_cube.mat');
    radar_cube = S.fft_complex_radar_cube;
    radar_cube = abs(radar_cube);
    velocities = S.velocities;
    limited_ranges = S.limited_ranges;
    range_idx = S.range_idx;
    [~, ~, numFrames, numRx] = size(radar_cube); % [numRangeBins, numDopplerBins, numFrames, numRx]

    % Data process
    % ranges = (1:numRangeBins-1) * range_res

    % Create figure and axes
    hFig = figure('Name','Range–Doppler FFT Viewer','NumberTitle','off');
    hAx  = axes('Parent',hFig, 'Position',[0.1 0.25 0.85 0.7]);

    % Dropdown (popup) menu for RX channel selection
    rxList = arrayfun(@(x)sprintf('RX %d',x), 1:numRx, 'UniformOutput',false);
    hPopup = uicontrol( ...
        'Style','popupmenu', ...
        'String',rxList, ...
        'Value',1, ...
        'Units','normalized', ...
        'Position',[0.05 0.05 0.2 0.05], ...
        'Callback',@updatePlot);

    % Slider for frame selection
    hSlider = uicontrol( ...
        'Style','slider', ...
        'Min',1, ...
        'Max',numFrames, ...
        'Value',1, ...
        'SliderStep',[1/(numFrames-1) , 10/(numFrames-1)], ...
        'Units','normalized', ...
        'Position',[0.3 0.05 0.5 0.05], ...
        'Callback',@updatePlot);

    % Text label to show current frame number
    hTxt = uicontrol( ...
        'Style','text', ...
        'Units','normalized', ...
        'Position',[0.82 0.05 0.12 0.05], ...
        'String','Frame: 1');

    % Initial plot
    updatePlot();

    function updatePlot(~,~)
        % Read UI values
        rx    = get(hPopup,'Value');
        frame   = round(get(hSlider,'Value'));
        set(hTxt,'String',sprintf('Frame: %d',frame));

        % Extract the RD FFT slice for (range × doppler)
        rdSlice = fliplr(radar_cube(range_idx,:, frame, rx));
        % rdSlice = 20*log10(fliplr(radar_cube(range_idx,:, frame, rx)));
        % frame_data = 20*log10(fliplr(norm_fft(range_idx,:)));

        % Display
        imagesc(hAx, velocities, limited_ranges, rdSlice);
        set(hAx,'YDir','normal');
        xlabel(hAx,'Doppler Bin');
        ylabel(hAx,'Range Bin');
        title(hAx,sprintf('RD FFT — RX %d, Frame %d', rx, frame));
        colorbar('peer',hAx);
    end
end
