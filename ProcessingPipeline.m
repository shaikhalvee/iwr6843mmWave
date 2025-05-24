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

        function cube1D = computeRangeFFT(obj, rawFrame)
            % rawFrame: [C×R×S]
            [C,R,S] = size(rawFrame);
            Nfft = 2^nextpow2(S);
            cube1D = zeros(C,R,Nfft);
            for c=1:C
                for r=1:R
                    sig = squeeze(rawFrame(c,r,:)) .* obj.device.win_hann;
                    cube1D(c,r,:) = fft(sig, Nfft);
                end
            end
        end

        function cube2D = computeRangeDopplerFFT(obj, rawFrame, chanIdx)
            % rawFrame: [C×R×S], chanIdx: RX channel index
            data2D = squeeze(rawFrame(:,chanIdx,:));  % [C×S]
            % DC removal
            data2D = data2D - mean(data2D,2);
            % 2D window
            win2D = hamming(size(data2D,1)) * obj.device.win_hann';
            data2D = data2D .* win2D;
            % FFT along both dims & shift Doppler
            rd = fftshift(fft(fft(data2D,[],1),[],2),2);
            cube2D = abs(rd);
        end

        function exportAll(obj, saveRaw, save1D, save2D)
            F = obj.device.total_frames;
            R = 2^nextpow2(obj.device.num_adc_sample_per_chirp);
            D = obj.device.num_chirp_per_frame;
            X = obj.device.num_rx;
            
            outputDir = fullfile('output', obj.device.testRootPath);
            if ~exist(outputDir,'dir')
                mkdir(outputDir);
            end
            saveRaw = fullfile(outputDir, saveRaw);
            save1D = fullfile(outputDir, save1D);
            save2D = fullfile(outputDir, save2D);

            % Preallocate
            adcBinRaw   = cell(1,F);
            rangeFFTData = cell(1,F);
            rangeDopplerFFTData = zeros(R, D, F, X);

            for f=1:F
                rawFrame = obj.device.readRawFrame(f);
                adcBinRaw{f} = rawFrame;
                rangeFFTData{f} = obj.computeRangeFFT(rawFrame);
                for x=1:X
                    rangeDopplerFFTData(:,:,f,x) = obj.computeRangeDopplerFFT(rawFrame, x);
                end
            end

            if ~isempty(saveRaw)
                save(saveRaw, 'adcBinRaw','-v7.3');
            end
            if ~isempty(save1D)
                save(save1D,  'rangeFFTData','-v7.3');
            end
            if ~isempty(save2D)
                % Also save axes                
                range_res = obj.device.range_res;
                v_max = obj.device.v_max;
                v_res = obj.device.v_res;
                velocities = -v_max:v_res:(v_max-v_res);
                save(save2D,'rangeDopplerFFTData','range_res', ...
                    'v_res', 'velocities','-v7.3');
            end
        end
    end
end