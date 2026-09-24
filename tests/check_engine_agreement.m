function check_engine_agreement()
%CHECK_ENGINE_AGREEMENT Same MATLAB equations under two integration engines.
% Run after run_tests(true). This is a stronger local gate than compile alone.
startup_project(); cfg=config_default(); cfg.T=2; cfg.visualize=false;
[model,cfg]=build_model(cfg); out=sim(model); ts=out.get('states');
ref=run_matlab_reference(cfg);
A=reshape(ts.Data,numel(ts.Time),[]); B=ref.raw.states;
assert(isequal(size(A),size(B)));
assert(max(abs(ts.Time(:)-ref.raw.t))<1e-9);
% Small differences are permitted at sampled-control boundary evaluations.
error=max(abs(A(:)-B(:)));
fprintf('Maximum absolute state discrepancy across engines: %.6g\n',error);
assert(error<1e-4,'Integration engines disagree; inspect sample-hit scheduling.');
end
