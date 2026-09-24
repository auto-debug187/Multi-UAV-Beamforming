function result=run_demo(cfg)
%RUN_DEMO Build, run, visualize and export one Simulink experiment.
root=startup_project(); if nargin==0, cfg=config_default(); end
[model,cfg]=build_model(cfg); open_system(model);
out=sim(model);
ts=out.get('states'); ms=out.get('motors'); ds=out.get('diagnostics');
raw.t=ts.Time(:); raw.states=reshape(ts.Data,numel(raw.t),[]);
raw.motors=reshape(ms.Data,numel(raw.t),[]);
raw.diagnostics=reshape(ds.Data,numel(raw.t),[]);
result=bf_evaluate(raw,cfg); result.engine='Simulink ode4';
folder=fullfile(root,'results',sprintf('%s_N%d',cfg.mode,cfg.N));
bf_export(result,folder); bf_plot_results(result,folder);
fprintf('\nResults: %s\n',folder); disp(struct2table(result.summary));
end
