function bf_plot_results(r,folder)
%BF_PLOT_RESULTS Export static performance plots and a final array-factor cut.
t=r.raw.t; cfg=r.cfg;
f=figure('Color','w','Name','Formation performance','Position',[80 80 1250 780]);
tiledlayout(3,2,'TileSpacing','compact');
nexttile; plot(t,r.centroidError,'LineWidth',1.2); ylabel('Centroid error (m)'); grid on;
nexttile; plot(t,r.pointingDeg,'LineWidth',1.2); ylabel('Pointing error (deg)'); grid on;
nexttile; plot(t,r.edgeRMS,t,r.edgeMax,'LineWidth',1.2); ylabel('Edge error (m)'); legend('RMS','Maximum'); grid on;
nexttile; plot(t,r.minSeparation,'LineWidth',1.2); yline(cfg.minSeparation,'r--'); ylabel('Minimum separation (m)'); grid on;
nexttile; plot(t,r.snrDb,t,r.fixedPlaneSnrDb,'LineWidth',1.2); ylabel('SNR (dB)'); xlabel('Time (s)'); legend('Actual','Fixed-plane comparator'); grid on;
nexttile; plot(t,r.tiltDeg,'LineWidth',1.1); ylabel('Body tilt (deg)'); xlabel('Time (s)'); grid on;
sgtitle(sprintf('%s | %d drones | %s',cfg.mode,cfg.N,r.engine));
exportgraphics(f,fullfile(folder,'performance.png'),'Resolution',170);
f=figure('Color','w','Name','Actual trajectories','Position',[110 110 900 720]);
ax=axes(f); hold(ax,'on'); grid(ax,'on'); colors=lines(cfg.N);
for i=1:cfg.N
    j=17*(i-1); plot3(ax,r.raw.states(:,j+1),r.raw.states(:,j+2),r.raw.states(:,j+3), ...
        'Color',colors(i,:),'LineWidth',1.2,'DisplayName',sprintf('Drone %d',i));
end
C=r.desiredCentroid; plot3(ax,C(:,1),C(:,2),C(:,3),'k--','DisplayName','Desired centroid');
P=cfg.receiver; plot3(ax,P(1),P(2),P(3),'kp','MarkerFaceColor',[1 .6 0],'MarkerSize',15,'DisplayName','Receiver');
axis(ax,'equal'); view(ax,40,24); xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); legend('Location','best');
exportgraphics(f,fullfile(folder,'trajectories.png'),'Resolution',170);
f=figure('Color','w','Name','Far-field array factor slice');
X=reshape(r.raw.states(end,:),17,cfg.N); P=X(1:3,:); pc=mean(P,2);
ref=bf_reference(t(end),cfg); angles=linspace(-pi,pi,721); dirs=ref.R*[sin(angles);zeros(size(angles));cos(angles)];
af=abs(sum(exp(-1i*2*pi/cfg.lambda*((P-pc)'*dirs)+1i*cfg.rf.phaseOffsets),1)).^2/cfg.N^2;
plot(angles*180/pi,10*log10(max(af,1e-8)),'LineWidth',1.3); grid on; ylim([-50 0]);
xlabel('Angle from desired normal in a formation-frame slice (deg)'); ylabel('Normalized power (dB)');
title('Far-field array-factor slice: equal amplitudes; front/back symmetry');
exportgraphics(f,fullfile(folder,'array_factor_slice.png'),'Resolution',170);
end
