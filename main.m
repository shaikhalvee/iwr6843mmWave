%% File: main.m
% Entry point for unified pipeline
setupFile = "D:\Documents\Drone_Data\single_chip\drone_21_5_2025_60ms\capture_config.setup.json";   % Setup JSON (includes bin list & config)

% Outputs
saveRaw = 'rawData.mat';
save1D  = 'rangeFFT.mat';
save2D  = 'rangeDopplerMap.mat';

% Instantiate device & pipeline
device   = iwr6843mmWaveDevice(setupFile);
pipeline = ProcessingPipeline(device);

% Export data
pipeline.exportAll(saveRaw, save1D, save2D);

% Launch GUI
launchPostProcessingGUI(pipeline);

% Cleanup
device.closeFiles();
