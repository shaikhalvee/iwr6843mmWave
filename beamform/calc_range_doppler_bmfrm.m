function [RD_map, range_axis, doppler_axis] = calc_range_doppler_bmfrm(adc_cube, paramsConfig, BF_MIMO_ref)
    % Windowing
    num_samples = paramsConfig.Samples_per_Chirp;
    num_chirps  = paramsConfig.nchirp_loops;
    num_rx      = paramsConfig.numRX;

    range_win   = hann(num_samples);
    doppler_win = hann(num_chirps);

    % Range FFT (dim 1)
    adc_cube_win = adc_cube .* reshape(range_win,[],1,1,1);
    range_fft    = fft(adc_cube_win, paramsConfig.rangeFFTSize, 1);

    % Doppler FFT (dim 2)
    range_fft_win = range_fft .* reshape(doppler_win,1,[],1,1);
    doppler_fft   = fftshift(fft(range_fft_win, paramsConfig.nchirp_loops, 2), 2);

    % RX calibration
    for rx = 1:num_rx
        doppler_fft(:,:,rx,:) = doppler_fft(:,:,rx,:) * exp(-1i * BF_MIMO_ref(rx) * pi/180);
    end

    % Combine RX (max or sum, single angle so no angle FFT)
    RD_map = squeeze(sum(abs(doppler_fft), 3)); % [range, doppler]

    % Axes
    c = 3e8;
    fs = paramsConfig.Sampling_Rate_ksps * 1e3;
    slope = paramsConfig.Slope_MHzperus * 1e12;
    range_axis = (0:paramsConfig.rangeFFTSize-1) * c * fs / (2 * slope * paramsConfig.rangeFFTSize);
    PRF = 1e6 / (paramsConfig.Idle_Time_us + paramsConfig.Ramp_End_Time_us);
    v_max = c * paramsConfig.Slope_MHzperus * 1e6 / (2 * paramsConfig.Start_Freq_GHz * 1e9);
    doppler_axis = linspace(-v_max, v_max, paramsConfig.nchirp_loops);
end
