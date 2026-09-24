function out=bf_evaluate(raw,cfg)
%BF_EVALUATE Metrics are calculated from actual states, not setpoint geometry.
out.cfg=cfg; out.raw=raw; t=raw.t; M=numel(t); N=cfg.N;
assert(all(isfinite(raw.states(:))),'Nonfinite plant state.');
out.centroid=zeros(M,3); out.desiredCentroid=zeros(M,3);
out.positionError=zeros(M,N); out.centroidError=zeros(M,1);
out.pointingDeg=zeros(M,1); out.axisErrorDeg=zeros(M,1);
out.edgeRMS=zeros(M,1); out.edgeMax=zeros(M,1); out.minSeparation=zeros(M,1);
out.planarity=zeros(M,1); out.tiltDeg=zeros(M,N); out.speed=zeros(M,N);
out.quatNormError=zeros(M,N); out.snrDb=nan(M,1); out.coherentLossDb=nan(M,1);
out.fixedPlaneSnrDb=nan(M,1); out.farFieldRatio=zeros(M,1);
initial=bf_reference(0,cfg); Rfixed=initial.R;
for k=1:M
    X=reshape(raw.states(k,:),17,N); P=X(1:3,:); V=X(4:6,:);
    ref=bf_reference(t(k),cfg); pc=mean(P,2); [R,~,rel]=bf_formationPose(P,V,cfg);
    los=cfg.receiver-pc; los=los/norm(los); ct=max(-1,min(1,dot(R(:,3),los)));
    out.centroid(k,:)=pc'; out.desiredCentroid(k,:)=ref.c';
    out.centroidError(k)=norm(pc-ref.c);
    out.positionError(k,:)=vecnorm(P-ref.p,2,1);
    out.pointingDeg(k)=acosd(ct); out.axisErrorDeg(k)=acosd(abs(ct));
    ds=zeros(size(cfg.edges,1),1);
    for j=1:numel(ds), ds(j)=norm(P(:,cfg.edges(j,1))-P(:,cfg.edges(j,2))); end
    de=ds-cfg.edgeLength;
    out.edgeRMS(k)=sqrt(mean(de.^2)); out.edgeMax(k)=max(abs(de));
    out.minSeparation(k)=min(ds); out.planarity(k)=sqrt(mean((R(:,3)'*rel).^2));
    out.speed(k,:)=vecnorm(V,2,1);
    for i=1:N
        Ri=bf_quatR(X(7:10,i)); out.tiltDeg(k,i)=acosd(max(-1,min(1,Ri(3,3))));
        out.quatNormError(k,i)=abs(norm(X(7:10,i))-1);
    end
    if cfg.rf.enabled
        rf=bf_rf(P,cfg); out.snrDb(k)=rf.snr; out.coherentLossDb(k)=rf.loss;
        base=bf_rf(pc+Rfixed*cfg.rho,cfg); out.fixedPlaneSnrDb(k)=base.snr;
    end
    % Fraunhofer aperture criterion, diagnostic only; exact ranges are used above.
    aperture=max(ds); out.farFieldRatio(k)=norm(cfg.receiver-pc)/(2*aperture^2/cfg.lambda);
end
diagnostics=reshape(raw.diagnostics',4,N,M);
out.accelLimited=reshape(diagnostics(1,:,:),N,M)';
out.motorSaturated=reshape(diagnostics(2,:,:),N,M)';
settled=t>=min(10,cfg.T/2); % exclude first 10 s, or half of a shorter run
s=struct();
s.centroidRMSE_m=sqrt(mean(out.centroidError.^2));
s.centroidMax_m=max(out.centroidError);
s.dronePositionRMSE_m=sqrt(mean(out.positionError(:).^2));
s.pointingRMSE_deg=sqrt(mean(out.pointingDeg.^2));
s.pointingMax_deg=max(out.pointingDeg);
s.settledPointingRMSE_deg=sqrt(mean(out.pointingDeg(settled).^2));
s.edgeRMSE_m=sqrt(mean(out.edgeRMS.^2));
s.edgeMax_m=max(out.edgeMax); s.minimumSeparation_m=min(out.minSeparation);
s.maximumPlanarityError_m=max(out.planarity);
s.maximumSpeed_mps=max(out.speed(:)); s.maximumTilt_deg=max(out.tiltDeg(:));
s.accelerationLimited_percent=100*mean(out.accelLimited(:));
s.motorSaturated_percent=100*mean(out.motorSaturated(:));
s.quaternionNormMaxError=max(out.quatNormError(:));
s.minimumFarFieldRatio=min(out.farFieldRatio);
s.coherentLossMean_dB=mean(out.coherentLossDb,'omitnan');
s.snrMean_dB=mean(out.snrDb,'omitnan');
s.snrGainOverFixedPlaneMean_dB=mean(out.snrDb-out.fixedPlaneSnrDb,'omitnan');
s.separationViolation=any(out.minSeparation<cfg.minSeparation);
inside=out.pointingDeg<2 & out.centroidError<0.1 & out.edgeMax<0.05;
lastBad=find(~inside,1,'last');
if isempty(lastBad), s.settlingTime_s=t(1);
elseif lastBad==M, s.settlingTime_s=NaN;
else, s.settlingTime_s=t(lastBad+1); end
out.summary=s;
end
