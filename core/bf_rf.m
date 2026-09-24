function rf=bf_rf(P,cfg)
%BF_RF Spherical-wave LOS Friis amplitudes, coherent narrowband superposition.
% No virtual transmitter. Common carrier phase cancels in the power magnitude.
d=vecnorm(cfg.receiver-P,2,1)';
assert(all(d>0),'Receiver coincides with a transmitter.');
A=sqrt(cfg.rf.powerPerDrone)*cfg.lambda./(4*pi*d);
phasor=A.*exp(1i*(cfg.rf.phaseOffsets-2*pi*d/cfg.lambda));
rf.power=abs(sum(phasor))^2;
rf.bound=sum(A)^2; % oracle: perfect phase alignment at these distances
rf.efficiency=rf.power/max(rf.bound,realmin);
rf.snr=10*log10(max(rf.power,realmin)/cfg.rf.noisePower);
rf.loss=-10*log10(max(rf.efficiency,realmin));
end
