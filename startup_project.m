function root = startup_project()
%STARTUP_PROJECT Add only this project's source directories to the path.
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root,'core'), fullfile(root,'simulink'), ...
    fullfile(root,'visualization'), fullfile(root,'tests'));
if ~exist(fullfile(root,'models'),'dir'), mkdir(fullfile(root,'models')); end
if ~exist(fullfile(root,'results'),'dir'), mkdir(fullfile(root,'results')); end
end
