% iwr6843mmWaveDevice

classdef iwr6843mmWaveDevice
    % Handles JSON parsing, unit conversion, multi‑file .bin I/O
    properties
        % set up files
        setupJSON        % setup JSON struct (list of .bin files + mmWave JSON path)
        mmwaveJSON       % TI mmWave JSON struct
        
        testBasePath     % name of the file base path of the test files
        testRootPath     % name of the test root file/folder
        binFileNames     % cell array of .bin file paths
        fidBin           % file IDs for each .bin
        numFramePerFile  % frames in each file
        
        % Data placeholders
        adc_raw_bin               % Raw ADC data vector for last read

        % Radar configuration & derived parameters from mmWave JSON
        start_freq                % Center frequency (Hz)
        lambda                    % Wavelength (m)
        aoa_angle                 % AOA search grid (degrees)
        num_angle                 % Number of angles

        chirp_slope               % Frequency slope (MHz/µs)
        bandwidth                 % Chirp bandwidth (MHz)
        chirp_ramp_time           % Ramp time (µs)
        chirp_idle_time           % Idle time between chirps (µs)
        chirp_adc_start_time      % ADC start time offset (µs)
        frame_periodicity         % Frame period (ms)

        total_frames              % Total frames across all .bin files
        num_sample_per_frame      % Samples per frame (all Rx)
        size_per_frame            % Bytes per frame in .bin
        num_adc_sample_per_chirp  % ADC samples per chirp
        num_chirp_per_frame       % Chirps per frame

        adc_samp_rate             % ADC sampling rate (Msps)
        adc_bits                  % ADC resolution (bits)
        dbfs_coeff                % Digital full-scale correction (dBFS)

        range_max                 % Maximum unambiguous range (m)
        range_res                 % Range resolution (m)
        v_max                     % Maximum unambiguous velocity (m/s)
        v_res                     % Velocity resolution (m/s)

        rx_chanl_enable           % RX channel enable mask (vector)
        num_rx                    % Number of RX channels
        num_byte_per_sample       % Bytes per ADC sample
        win_hann                  % Hanning window for ADC samples

        is_iq_swap                % Flag: IQ swap enabled
        is_interleave             % Flag: channel interleaving enabled

    end
    methods
        function obj = iwr6843mmWaveDevice(setupJsonFile)
            % Constructor reads setup JSON, mmWave JSON, and prepares bin files
            obj.setupJSON = jsondecode(fileread(setupJsonFile));
            
            % Read mmWave JSON referenced in setup
            mmwFile = obj.setupJSON.configUsed;
            sys_json = jsondecode(fileread(mmwFile));
            obj.mmwaveJSON = sys_json;

            % Parse mmWave profile, frame, channel, ADC format
            profileConfig = sys_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t;
            frameConfig = sys_json.mmWaveDevices.rfConfig.rlFrameCfg_t;
            channelConfig = sys_json.mmWaveDevices.rfConfig.rlChanCfg_t;
            adcFormat = sys_json.mmWaveDevices.rfConfig.rlAdcOutCfg_t.fmt;
            devDatafmtCfg = sys_json.mmWaveDevices.rawDataCaptureConfig.rlDevDataFmtCfg_t;

            % RF parameters
            obj.start_freq           = profileConfig.startFreqConst_GHz * 1e9;
            obj.lambda               = physconst('LightSpeed') / obj.start_freq;
            obj.aoa_angle            = 0:1:180;
            obj.num_angle            = numel(obj.aoa_angle);

            % Samples & chirps
            obj.adc_samp_rate = profileConfig.digOutSampleRate / 1000;
            obj.num_adc_sample_per_chirp = profileConfig.numAdcSamples;
            obj.num_chirp_per_frame = frameConfig.numLoops;
            obj.win_hann = hanning(obj.num_adc_sample_per_chirp);

            obj.chirp_slope          = profileConfig.freqSlopeConst_MHz_usec;
            obj.bandwidth            = obj.chirp_slope * profileConfig.numAdcSamples / obj.adc_samp_rate;
            obj.chirp_ramp_time      = profileConfig.rampEndTime_usec;
            obj.chirp_idle_time      = profileConfig.idleTimeConst_usec;
            obj.chirp_adc_start_time = profileConfig.adcStartTimeConst_usec;
            obj.frame_periodicity    = frameConfig.framePeriodicity_msec;

            % ADC channel config
            obj.num_byte_per_sample = 4;
            mask = sscanf(channelConfig.rxChannelEn,'0x%x');
            obj.rx_chanl_enable     = bitget(mask,1:8);
            obj.num_rx         = sum(obj.rx_chanl_enable);

            % Frame size
            obj.size_per_frame       = obj.num_byte_per_sample * obj.num_rx * ...
                                       obj.num_adc_sample_per_chirp * obj.num_chirp_per_frame;

            % Setup .bin file list and open
            basePath = obj.setupJSON.capturedFiles.fileBasePath;
            obj.testBasePath = basePath;
            [~, obj.testRootPath, ~] = fileparts(basePath);
            files = obj.setupJSON.capturedFiles.files;
            cur_frame = 0;
            for i=1:numel(files)
                fn = fullfile(basePath, files(i).processedFileName);
                obj.binFileNames{i} = fn;
                obj.fidBin(i) = fopen(fn,'r');
                finfo = dir(fn);
                frames = floor(finfo.bytes / obj.size_per_frame);
                obj.numFramePerFile(i) = frames;
                cur_frame     = cur_frame + frames;
            end
            obj.total_frames = cur_frame;

            % ADC output config
            obj.adc_bits      = (adcFormat.b2AdcBits==2)*16 + (adcFormat.b2AdcBits~=2)*adcFormat.b2AdcBits;
            obj.dbfs_coeff    = - (20*log10(2^(obj.adc_bits-1)) + ...
                                20*log10(sum(obj.win_hann)) - 20*log10(sqrt(2)));

            % Range & velocity calculations
            obj.range_max = physconst('LightSpeed') * obj.adc_samp_rate*1e6 / ...
                             (2 * obj.chirp_slope*1e6 * 1e6);
            obj.range_res = physconst('LightSpeed') / (2 * obj.bandwidth*1e6);
            obj.v_max     = obj.lambda / (4*(obj.chirp_ramp_time + obj.chirp_idle_time)/1e6);
            obj.v_res     = obj.lambda / (2*obj.num_chirp_per_frame*(obj.chirp_idle_time + obj.chirp_ramp_time)/1e6);

            % Flags
            obj.is_iq_swap    = devDatafmtCfg.iqSwapSel;
            obj.is_interleave = devDatafmtCfg.chInterleave;
        end
        
        
        function rawFrame = readRawFrame(obj, frameIdx)
            % Reads a single frame's raw ADC data from the appropriate .bin
            idx = frameIdx;
            for i=1:numel(obj.numFramePerFile)
                if idx <= obj.numFramePerFile(i)
                    fid = obj.fidBin(i);
                    break;
                end
                idx = idx - obj.numFramePerFile(i);
            end
            offset = (idx-1) * obj.size_per_frame;
            fseek(fid, offset, 'bof');
            % Read int16, convert to complex IQ vector
            nElems = obj.num_chirp_per_frame * obj.num_rx * obj.num_adc_sample_per_chirp;
            data = fread(fid, nElems, 'int16');
            rawFrame = reshape(data, [obj.num_adc_sample_per_chirp, obj.num_rx, obj.num_chirp_per_frame]);
            rawFrame = permute(rawFrame, [3,2,1]);  % [chirp × rx × samples]
        end

        function closeFiles(obj)
            % Close all open file IDs
            for f = obj.fidBin, fclose(f); end
        end
    end
end
