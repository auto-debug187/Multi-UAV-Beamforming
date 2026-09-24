function run_tests(includeSimulink)
%RUN_TESTS Meaningful equation tests, then optional generated-model smoke tests.
% run_tests(false) requires MATLAB only. run_tests(true) also compiles/runs SLX.
if nargin==0, includeSimulink=true; end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); startup_project();
cfg=bf_prepare(config_default());
tol=2e-7;
for t=[0.2 2 4.9 5.1 15 35 59]
    r=bf_reference(t,cfg); h=1e-4;
    rp=bf_reference(t+h,cfg); rm=bf_reference(t-h,cfg);
    assert(norm(r.R'*r.R-eye(3),'fro')<1e-11 && det(r.R)>0.999999);
    assert(norm(r.R(:,3)-(cfg.receiver-r.c)/norm(cfg.receiver-r.c))<1e-12);
    assert(max(abs((rp.p(:)-rm.p(:))/(2*h)-r.v(:)))<tol);
    assert(max(abs((rp.v(:)-rm.v(:))/(2*h)-r.a(:)))<tol);
    assert(norm(mean(r.p,2)-r.c)<1e-12);
    [J,Jd,q]=bf_rigidity(r.p,r.v,cfg);
    assert(norm(q)<1e-11 && norm(J*r.v(:))<1e-11);
    [Jp,~,qp]=bf_rigidity(r.p+h*r.v,r.v,cfg);
    [Jm,~,qm]=bf_rigidity(r.p-h*r.v,r.v,cfg);
    assert(norm((qp-qm)/(2*h)-2*J*r.v(:))<tol);
    assert(norm((Jp-Jm)/(2*h)-Jd,'fro')<tol);
    X=cfg.x0; X(1:3,:)=r.p; X(4:6,:)=r.v;
    a=bf_formationControl(t,X,cfg); assert(norm(a-r.a,'fro')<1e-8);
    P=r.p; V=r.v; P(:,1)=P(:,1)+[.03;-.02;.04]; V(:,2)=V(:,2)+[.01;.02;-.03];
    [R,w]=bf_formationPose(P,V,cfg);
    Rp=bf_formationPose(P+h*V,V,cfg); Rm=bf_formationPose(P-h*V,V,cfg);
    assert(norm((Rp-Rm)/(2*h)-bf_skew(w)*R,'fro')<tol);
end
for v={[.1;-.2;.3],[0;0;0],[pi-1e-7;0;0]}
    R=bf_expSO3(v{1}); R2=bf_expSO3(bf_logSO3(R));
    assert(norm(R-R2,'fro')<1e-6);
end
x=cfg.x0(:,1); x(4:6)=0; x(7:10)=[1;0;0;0]; x(11:13)=0;
[u,d]=bf_autopilot(zeros(3,1),x,cfg); dx=bf_plantRHS(0,x,u,cfg);
assert(norm(dx)<1e-10,'Hover is not an equilibrium.');
assert(all(d(1:2)==0));
[u,d]=bf_autopilot([50;50;50],x,cfg);
assert(all(u>=0 & u<=cfg.vehicle.omegaMax) && d(1)==1);
r=bf_reference(12,cfg); rf=bf_rf(r.p,cfg);
assert(abs(rf.efficiency-1)<1e-12,'Aligned regular polygon should have equal ranges.');
cfg.rf.phaseOffsets=[0;2*pi/3;4*pi/3]; rf=bf_rf(r.p,cfg);
assert(rf.efficiency<1e-20,'Geometry must not hide independent carrier phase errors.');
for N=[3 6]
    c=config_default(); c.N=N; c=bf_prepare(c); r=bf_reference(8,c);
    assert(size(r.p,2)==N && numel(c.x0)==17*N);
end
fprintf('PASS: geometry, derivatives, rigidity, SO(3), hover, limits, RF, scaling.\n');
if includeSimulink
    modes={'geometric','paper'};
    for j=1:2
        c=config_default(); c.mode=modes{j}; c.T=1; c.visualize=false; c.pacing=false;
        [mdl,c]=build_model(c); out=sim(mdl); states=out.get('states');
        assert(abs(states.Time(end)-c.T)<1e-8);
        assert(all(isfinite(states.Data(:))));
        assert(size(states.Data,2)==17*c.N);
        close_system(mdl,0);
    end
    % Nonplanar initial perturbations and larger N must compile as well.
    c=config_default(); c.N=6; c.T=.2; c.visualize=false; c.initial.individualError=.03;
    [mdl,~]=build_model(c); out=sim(mdl); states=out.get('states');
    assert(all(isfinite(states.Data(:)))); close_system(mdl,0);
    fprintf('PASS: both Simulink modes and six-drone model compile and execute.\n');
end
end
