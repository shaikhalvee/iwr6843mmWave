%% File: main.m
% Entry point for unified pipeline
setupFile = "D:\Documents\Drone_Data\single_chip\drn_25_5_2025\capture.setup.json";   % Setup JSON (includes bin list & config)

[testRootFolder, ~, ~] = fileparts(setupFile);
[~, testRoot] = fileparts(testRootFolder);

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
% launchPostProcessingGUI(pipeline);
% rdFFTViewer(pipeline);
rdFFTViewer(testRoot)


% Cleanup
device.closeFiles();
