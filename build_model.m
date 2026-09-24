function [model,cfg]=build_model(cfg)
%BUILD_MODEL Generate an inspectable R2025a Simulink .slx from source.
% Only MATLAB and Simulink are needed; all nonlinear plant equations are open.
root=startup_project(); if nargin==0, cfg=config_default(); end
cfg=bf_prepare(cfg);
assert(license('test','Simulink'),'A Simulink license is required.');
load_system('simulink'); model='bf_swarm_generated';
if bdIsLoaded(model)
    assert(strcmp(get_param(model,'Dirty'),'off'), ...
        'Save your edits to the open generated model before rebuilding.');
    close_system(model,0);
end
new_system(model,'Model');
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep',num2str(cfg.dt,17),'StopTime',num2str(cfg.T,17), ...
    'SimulationMode','normal','ReturnWorkspaceOutputs','on', ...
    'SaveTime','off','SaveOutput','off','SignalLogging','off');
mw=get_param(model,'ModelWorkspace'); assignin(mw,'cfg',cfg);
if cfg.pacing, set_param(model,'EnablePacing','on','PacingRate','1'); end
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/Formation coordinator'],'FunctionName','sf_formation', ...
    'Parameters','cfg','Position',[90 90 285 160]);
add_block('simulink/Signal Routing/Demux',[model '/Acceleration commands'], ...
    'Outputs',mat2str(3*ones(1,cfg.N)),'Position',[340 95 345 110+120*cfg.N]);
add_block('simulink/Signal Routing/Mux',[model '/All states'], ...
    'Inputs',num2str(cfg.N),'Position',[790 85 795 110+120*cfg.N]);
add_block('simulink/Signal Routing/Mux',[model '/All motors'], ...
    'Inputs',num2str(cfg.N),'Position',[850 85 855 110+120*cfg.N]);
add_block('simulink/Signal Routing/Mux',[model '/All diagnostics'], ...
    'Inputs',num2str(cfg.N),'Position',[910 85 915 110+120*cfg.N]);
add_line(model,'Formation coordinator/1','Acceleration commands/1','autorouting','on');
for i=1:cfg.N
    name=sprintf('Drone %02d',i); sub=[model '/' name]; y=90+120*(i-1);
    add_block('built-in/Subsystem',sub,'Position',[425 y 715 y+80]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/World acceleration'], ...
        'Position',[25 68 55 82]);
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
        [sub '/Autopilot'],'FunctionName','sf_autopilot','Parameters','cfg', ...
        'Position',[125 45 285 120]);
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
        [sub '/6DOF and motors'],'FunctionName','sf_quadrotor', ...
        'Parameters',sprintf('cfg,%d',i),'Position',[370 40 545 110]);
    ports={'State','Motor command','Diagnostics'};
    for k=1:3
        add_block('simulink/Ports & Subsystems/Out1',[sub '/' ports{k}], ...
            'Port',num2str(k),'Position',[635 40+70*(k-1) 665 54+70*(k-1)]);
    end
    add_line(sub,'World acceleration/1','Autopilot/1','autorouting','on');
    add_line(sub,'Autopilot/1','6DOF and motors/1','autorouting','on');
    add_line(sub,'6DOF and motors/1','Autopilot/2','autorouting','on');
    add_line(sub,'6DOF and motors/1','State/1','autorouting','on');
    add_line(sub,'Autopilot/1','Motor command/1','autorouting','on');
    add_line(sub,'Autopilot/2','Diagnostics/1','autorouting','on');
    add_line(model,sprintf('Acceleration commands/%d',i),[name '/1'],'autorouting','on');
    add_line(model,[name '/1'],sprintf('All states/%d',i),'autorouting','on');
    add_line(model,[name '/2'],sprintf('All motors/%d',i),'autorouting','on');
    add_line(model,[name '/3'],sprintf('All diagnostics/%d',i),'autorouting','on');
end
add_line(model,'All states/1','Formation coordinator/1','autorouting','on');
names={'states','motors','diagnostics'}; sources={'All states','All motors','All diagnostics'};
for k=1:3
    dest=['Log ' names{k}];
    add_block('simulink/Sinks/To Workspace',[model '/' dest], ...
        'VariableName',names{k},'SaveFormat','Timeseries','SampleTime','cfg.logTs', ...
        'MaxDataPoints','inf','Position',[1020 65+75*k 1180 95+75*k]);
    add_line(model,[sources{k} '/1'],[dest '/1'],'autorouting','on');
end
if cfg.visualize
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
        [model '/Live 3D view'],'FunctionName','sf_liveview','Parameters','cfg', ...
        'Position',[1020 380 1180 440]);
    add_line(model,'All states/1','Live 3D view/1','autorouting','on');
end
ann=Simulink.Annotation(model,sprintf([ ...
    'WORLD +Z UP | body FLU | quaternion [w x y z]\n' ...
    '%d physical UAVs; virtual centroid is not a transmitter\n' ...
    '%s control | 100 Hz default autopilot | continuous nonlinear plants\n' ...
    'Double-click each UAV to inspect the autopilot and 6DOF plant.'],cfg.N,cfg.mode));
ann.Position=[90 25];
save_system(model,fullfile(root,'models',[model '.slx']));
set_param(model,'SimulationCommand','update'); % compile gate on the user's MATLAB
fprintf('Created and compiled: %s\n',fullfile(root,'models',[model '.slx']));
end
