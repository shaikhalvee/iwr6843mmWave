function paramsConfig = get_radar_config(adc_data_folder)
    matfile = fullfile(adc_data_folder, 'radar_params_capture.mat');
    if isfile(matfile)
        S = load(matfile);
        paramsConfig = S.params;
        fprintf('[Config] Loaded radar params from MAT file.\n');
    else
        % fallback: parse from JSON as before (old data)
        paramsConfig = parameter_gen_from_Jason(adc_data_folder, struct());
        fprintf('[Config] Parsed radar params from JSON file.\n');
    end
end
