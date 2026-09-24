function replay_result(matfile,speed)
%REPLAY_RESULT Replay a saved simulation.mat at a chosen approximate speed.
startup_project(); if nargin<2, speed=1; end
assert(speed>0); d=load(matfile,'result'); r=d.result; h=bf_live_init(r.cfg);
start=tic; t0=r.raw.t(1);
for k=1:3:numel(r.raw.t)
    if ~isgraphics(h.fig), break; end
    X=reshape(r.raw.states(k,:),17,r.cfg.N);
    bf_live_update(h,r.raw.t(k),X,r.cfg);
    delay=(r.raw.t(k)-t0)/speed-toc(start);
    if delay>0, pause(min(delay,0.2)); end
end
end
