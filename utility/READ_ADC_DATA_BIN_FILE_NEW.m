function ret_adc_data = READ_ADC_DATA_BIN_FILE_NEW(adc_data_bin_file, mmWave_device)
    % 1. Hardcoded read parameters:
    numADCSamples = 256; numADCBits = 16; numRX = 4; isReal = 0;

    fid = fopen(adc_data_bin_file,'r');
    adcData = fread(fid, 'int16');  fclose(fid);

    % 2. Bit-depth correction if not full 16-bit:
    if numADCBits ~= 16
      ... adjust negative overflow ...
    end

    % 3. Complex IQ reconstruction (interleaved):
    numChirps = numel(adcData)/2/numADCSamples/numRX;
    counter = 1;
    for i=1:4:numel(adcData)-1
      LVDS(counter)   = adcData(i)   + 1j*adcData(i+2);
      LVDS(counter+1) = adcData(i+1) + 1j*adcData(i+3);
      counter = counter + 2;
    end
    LVDS = reshape(LVDS, numADCSamples*numRX, numChirps).';
    
    % 4. Demultiplex per Rx channel:
    adcData = zeros(numRX, numChirps*numADCSamples);
    for ch = 1:numRX
      for k = 1:numChirps
        startIdx = (k-1)*numADCSamples + 1;
        adcData(ch, startIdx : k*numADCSamples) = ...
          LVDS(k, (ch-1)*numADCSamples + 1 : ch*numADCSamples);
      end
    end

    ret_adc_data = adcData;
end
