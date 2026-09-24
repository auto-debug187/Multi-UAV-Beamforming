function results=run_comparison(cfg)
%RUN_COMPARISON Same dynamics/initial conditions for both controller modes.
startup_project(); if nargin==0, cfg=config_default(); end
cfg.visualize=false; cfg.pacing=false;
modes={'geometric','paper'}; results=cell(1,2);
for k=1:2
    cfg.mode=modes{k}; results{k}=run_demo(cfg);
end
tableOut=[struct2table(results{1}.summary);struct2table(results{2}.summary)];
tableOut=addvars(tableOut,string(modes(:)),'Before',1,'NewVariableNames','mode');
root=fileparts(mfilename('fullpath'));
writetable(tableOut,fullfile(root,'results','comparison.csv')); disp(tableOut);
end
