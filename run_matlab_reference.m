function result=run_matlab_reference(cfg)
%RUN_MATLAB_REFERENCE The same .m equations, explicit sampled-control RK4.
% Useful independent integration path; it does not exercise Simulink callbacks.
root=startup_project(); if nargin==0, cfg=config_default(); end
cfg=bf_prepare(cfg); steps=round(cfg.T/cfg.dt);
assert(abs(steps*cfg.dt-cfg.T)<1e-9,'T must be a multiple of dt.');
controlStride=round(cfg.Ts/cfg.dt); logStride=round(cfg.logTs/cfg.dt);
visStride=round(cfg.visualTs/cfg.dt); count=floor(steps/logStride)+1;
raw.t=zeros(count,1); raw.states=zeros(count,17*cfg.N);
raw.motors=zeros(count,4*cfg.N); raw.diagnostics=zeros(count,4*cfg.N);
X=cfg.x0; U=zeros(4,cfg.N); D=U; k=0;
if cfg.visualize, h=bf_live_init(cfg); end
for step=0:steps
    t=step*cfg.dt;
    if mod(step,controlStride)==0
        a=bf_formationControl(t,X,cfg);
        for i=1:cfg.N, [U(:,i),D(:,i)]=bf_autopilot(a(:,i),X(:,i),cfg); end
    end
    if mod(step,logStride)==0
        k=k+1; raw.t(k)=t; raw.states(k,:)=X(:)';
        raw.motors(k,:)=U(:)'; raw.diagnostics(k,:)=D(:)';
    end
    if cfg.visualize && mod(step,visStride)==0, bf_live_update(h,t,X,cfg); end
    if step==steps, break; end
    dt=cfg.dt;
    for i=1:cfg.N
        x=X(:,i); u=U(:,i);
        k1=bf_plantRHS(t,x,u,cfg);
        k2=bf_plantRHS(t+dt/2,x+dt/2*k1,u,cfg);
        k3=bf_plantRHS(t+dt/2,x+dt/2*k2,u,cfg);
        k4=bf_plantRHS(t+dt,x+dt*k3,u,cfg);
        X(:,i)=x+dt/6*(k1+2*k2+2*k3+k4);
    end
end
result=bf_evaluate(raw,cfg); result.engine='MATLAB explicit RK4';
folder=fullfile(root,'results',sprintf('matlab_%s_N%d',cfg.mode,cfg.N));
bf_export(result,folder); bf_plot_results(result,folder);
disp(struct2table(result.summary));
end
