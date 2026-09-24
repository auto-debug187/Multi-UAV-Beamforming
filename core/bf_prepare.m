function cfg = bf_prepare(cfg)
%BF_PREPARE Validate configuration and derive dimensions and initial states.
assert(cfg.N>=3 && cfg.N==floor(cfg.N),'N must be an integer >=3.');
assert(any(strcmp(cfg.mode,{'paper','geometric'})),'Unknown controller mode.');
assert(cfg.radius>0 && cfg.dt>0 && cfg.Ts>0 && cfg.T>0);
for h = [cfg.Ts cfg.logTs cfg.visualTs]
    assert(abs(h/cfg.dt-round(h/cfg.dt))<1e-8, ...
        'All sample periods must be integer multiples of dt.');
end
assert(cfg.path.ramp>0 && cfg.path.period>0);
assert(cfg.vehicle.mass>0 && all(eig(cfg.vehicle.I)>0));
assert(cfg.vehicle.motorTau>0 && cfg.vehicle.kT>0 && cfg.vehicle.kQ>0);
assert(cfg.control.maxVerticalAccel < cfg.vehicle.g);
assert(cfg.control.maxTilt>0 && cfg.control.maxTilt<pi/2);
assert(numel(cfg.receiver)==3 && all(isfinite(cfg.receiver)));
cfg.receiver=cfg.receiver(:); cfg.frameSeed=cfg.frameSeed(:)/norm(cfg.frameSeed);
a=2*pi*(0:cfg.N-1)/cfg.N;
cfg.rho=cfg.radius*[cos(a);sin(a);zeros(1,cfg.N)];
cfg.edges=nchoosek(1:cfg.N,2); % complete graph, redundant for N>3
cfg.edgeLength=zeros(size(cfg.edges,1),1);
for k=1:size(cfg.edges,1)
    cfg.edgeLength(k)=norm(cfg.rho(:,cfg.edges(k,1))-cfg.rho(:,cfg.edges(k,2)));
end
cfg.minSeparation=2*cfg.vehicle.bodyRadius;
assert(min(cfg.edgeLength)>cfg.minSeparation, ...
    'Formation too small for drone rotor envelopes. Increase radius or shrink vehicle.');
l=cfg.vehicle.arm/sqrt(2);
cfg.vehicle.rotorXY=[l -l -l l; l l -l -l];
cfg.vehicle.spin=[1 -1 1 -1];
cfg.vehicle.B=[ones(1,4);cfg.vehicle.rotorXY(2,:); ...
    -cfg.vehicle.rotorXY(1,:);cfg.vehicle.kQ/cfg.vehicle.kT*cfg.vehicle.spin];
cfg.vehicle.Binv=inv(cfg.vehicle.B);
cfg.vehicle.fMax=cfg.vehicle.kT*cfg.vehicle.omegaMax^2;
assert(4*cfg.vehicle.fMax>cfg.vehicle.mass*cfg.vehicle.g);
cfg.lambda=cfg.rf.c/cfg.rf.frequency;
if isempty(cfg.rf.phaseOffsets), cfg.rf.phaseOffsets=zeros(cfg.N,1); end
assert(numel(cfg.rf.phaseOffsets)==cfg.N);
cfg.rf.phaseOffsets=cfg.rf.phaseOffsets(:);
ref=bf_reference(0,cfg);
initialR=bf_expSO3(cfg.initial.formationRotation)*ref.R;
cfg.x0=zeros(17,cfg.N);
for i=1:cfg.N
    perturb=cfg.initial.individualError*[cos(a(i));sin(2*a(i));sin(a(i))];
    cfg.x0(1:3,i)=ref.c+cfg.initial.centroidError+initialR*cfg.rho(:,i)+perturb;
    cfg.x0(7,i)=1;                % body initially level
    cfg.x0(14:17,i)=sqrt(cfg.vehicle.mass*cfg.vehicle.g/(4*cfg.vehicle.kT));
end
% Check geometric feasibility before starting a potentially long simulation.
for t=linspace(0,cfg.T,301)
    r=bf_reference(t,cfg);
    assert(min(r.p(3,:))>0.5,'Reference formation gets too close to ground.');
end
if cfg.radius>cfg.lambda/2
    warning('bf:spacing','Radius exceeds lambda/2. Broadside still exists; extra lobes may occur.');
end
end
