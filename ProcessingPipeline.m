%% File: ProcessingPipeline.m

classdef ProcessingPipeline
    % Encapsulates 1D range-FFT, 2D Range–Doppler FFT, and data export
    properties
        device            % MMWaveDevice instance
    end
    methods
        function obj = ProcessingPipeline(device)
            obj.device = device;
        end

        function [rangeFFT, rdFFT] = computeRangeDoppler(obj, rawFrame)
            % rawFrame: [R × D × Rx]
            [R, D, Rx] = size(rawFrame);

            % zero-pad range to next-power-of-2
            NfftR = 2^nextpow2(R);

            % pre-allocate
            rangeFFT = zeros(NfftR, D, Rx);
            rdFFT    = zeros(NfftR, D, Rx);

            % build 2-D window: hanning in range, hanning in Doppler (chirps)
            winRange   = obj.device.range_hann;      % [R×1]
            winDoppler = obj.device.dopp_hann;       % [D×1]
            win2D      = winRange * winDoppler.';  % [R×D]

            for rx = 1:Rx
                % pull out each Rx’s [R×D] block
                data2D = squeeze(rawFrame(:,:,rx)); % [RxD]

                % % DC subtraction
                data2D = data2D - mean(data2D, 2);

                % apply 2D window
                dataWin = data2D .* win2D;

                % Range FFT along dim-1
                rangeFFT(:,:,rx) = fft(dataWin, NfftR, 1);

                % Doppler FFT along dim-2 + shift zero-Doppler to center
                rdFFT(:,:,rx) = fftshift( fft(rangeFFT(:,:,rx), [], 2), 2 );
            end
        end

        function exportAll(obj, saveRaw, save1D, save2D)
            F = obj.device.total_frames;
            R = 2^nextpow2(obj.device.num_adc_sample_per_chirp);
            D = obj.device.num_chirp_per_frame;
            X = obj.device.num_rx_chnl;

            % save out
            outputDir = fullfile('output', obj.device.testRootPath);
            if ~exist(outputDir,'dir'), mkdir(outputDir); end

            %--- full paths to save files ---
            rawPath  = fullfile(outputDir, saveRaw);
            fft1DPath= fullfile(outputDir, save1D);
            rd2DPath = fullfile(outputDir, save2D);

            % pre-allocate storage for Range-Doppler directly
            adcBinRaw            = cell(1, F);
            rangeFFTData         = cell(1, F);
            rangeDopplerFFTData  = zeros(R, D, X, F, 'single');


            for f = 1:F
                rawFrame = obj.device.readRawFrame(f);
                adcBinRaw{f} = rawFrame;

                % compute both at once
                [rangeFFTFrame, rdFFTFrame] = obj.computeRangeDoppler(rawFrame);

                % store
                rangeFFTData{f} = rangeFFTFrame;
                rangeDopplerFFTData(:,:,:,f) = single(rdFFTFrame);
            end

           
            if ~isempty(saveRaw)
                save(rawPath, 'adcBinRaw', '-v7.3');
            end
            if ~isempty(save1D)
                save(fft1DPath, 'rangeFFTData', '-v7.3');
            end
            if ~isempty(save2D)
                % also export axes
                range_res  = obj.device.range_res;
                v_res      = obj.device.v_res;
                v_max      = obj.device.v_max;
                velocities = -v_max : v_res : (v_max - v_res);
                save(rd2DPath, 'rangeDopplerFFTData', 'range_res', ...
                    'v_res', 'velocities', '-v7.3');
            end
        end
    end
end
