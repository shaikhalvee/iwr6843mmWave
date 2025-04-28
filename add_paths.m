clear all;

ignoreFolders = {'.git', '__MACOSX', '.svn', '.idea', '.vscode', 'private'};

currentFile = mfilename('fullpath');
projectRoot = fileparts(currentFile);

folders = genpath(projectRoot);
folders = strsplit(folders, pathsep);

% Exclude '.idea' and empty entries
folders = folders(~cellfun(@isempty, folders)); % remove empty first
folders = folders(~contains(folders, ignoreFolders)); % remove .idea folders

% Now add paths safely
addpath(folders{:});
