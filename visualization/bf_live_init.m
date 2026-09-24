function h=bf_live_init(cfg)
h.fig=figure('Name','Beamforming formation: live nonlinear simulation', ...
    'Color','w','Position',[60 60 1350 780]);
layout=tiledlayout(h.fig,2,2,'TileSpacing','compact');
h.ax=nexttile(layout,1,[2 1]); hold(h.ax,'on'); grid(h.ax,'on'); axis(h.ax,'equal');
view(h.ax,40,24); xlabel(h.ax,'World x (m)'); ylabel(h.ax,'World y (m)'); zlabel(h.ax,'World z (m)');
tt=linspace(0,cfg.T,400); path=zeros(3,numel(tt));
for k=1:numel(tt), r=bf_reference(tt(k),cfg); path(:,k)=r.c; end
plot3(h.ax,path(1,:),path(2,:),path(3,:),'k--','LineWidth',1,'DisplayName','Centroid reference');
rec=cfg.receiver;
plot3(h.ax,rec(1),rec(2),rec(3),'p','MarkerSize',16,'MarkerFaceColor',[1 .65 .1], ...
    'MarkerEdgeColor','k','DisplayName','Fixed receiver');
text(h.ax,rec(1),rec(2),rec(3)+.5,'Receiver','FontWeight','bold');
pad=2; xmin=min([path(1,:) rec(1)])-pad; xmax=max([path(1,:) rec(1)])+pad;
ymin=min([path(2,:) rec(2)])-pad; ymax=max([path(2,:) rec(2)])+pad;
patch(h.ax,[xmin xmax xmax xmin],[ymin ymin ymax ymax],[0 0 0 0], ...
    [.92 .94 .96],'FaceAlpha',.5,'EdgeColor',[.8 .8 .8],'HandleVisibility','off');
xlim(h.ax,[xmin xmax]); ylim(h.ax,[ymin ymax]); zlim(h.ax,[0 max(rec(3)+1,max(path(3,:))+3)]);
h.plane=patch(h.ax,nan(1,cfg.N),nan(1,cfg.N),nan(1,cfg.N), ...
    [.15 .60 .8],'FaceAlpha',.18,'EdgeColor',[.1 .4 .6],'DisplayName','Actual formation');
h.target=plot3(h.ax,nan,nan,nan,'k--','LineWidth',1.2,'DisplayName','Desired formation');
h.normal=plot3(h.ax,nan,nan,nan,'r-','LineWidth',2,'DisplayName','Actual normal');
h.los=plot3(h.ax,nan,nan,nan,'g:','LineWidth',1.4,'DisplayName','Line to receiver');
h.centroid=plot3(h.ax,nan,nan,nan,'ko','MarkerFaceColor','k','DisplayName','Virtual centroid');
colors=lines(cfg.N); h.trace=gobjects(1,cfg.N); h.arm=gobjects(2,cfg.N); h.label=gobjects(1,cfg.N);
for i=1:cfg.N
    h.trace(i)=animatedline(h.ax,'Color',colors(i,:),'LineWidth',1.2, ...
        'MaximumNumPoints',5000,'DisplayName',sprintf('Drone %d',i));
    for j=1:2
        h.arm(j,i)=plot3(h.ax,nan,nan,nan,'-o','Color',colors(i,:), ...
            'LineWidth',2.5,'MarkerSize',4,'HandleVisibility','off');
    end
    h.label(i)=text(h.ax,0,0,0,sprintf(' %d',i),'Color',colors(i,:));
end
legend(h.ax,'Location','southoutside','NumColumns',2);
h.title=title(h.ax,sprintf('%s controller | N=%d',cfg.mode,cfg.N));
ax=nexttile(layout,2); hold(ax,'on'); grid(ax,'on'); xlim(ax,[0 cfg.T]);
h.pointing=animatedline(ax,'Color',[.8 .15 .15],'LineWidth',1.4);
ylabel(ax,'Pointing error (deg)'); xlabel(ax,'Simulation time (s)'); title(ax,'Actual normal versus actual centroid LOS');
ax=nexttile(layout,4); hold(ax,'on'); grid(ax,'on'); xlim(ax,[0 cfg.T]);
h.centerError=animatedline(ax,'Color',[.1 .3 .8],'LineWidth',1.3,'DisplayName','Centroid error');
h.edgeError=animatedline(ax,'Color',[.1 .6 .3],'LineWidth',1.3,'DisplayName','Edge RMS');
ylabel(ax,'Error (m)'); xlabel(ax,'Simulation time (s)'); legend(ax,'Location','best');
title(ax,'Tracking and shape'); setappdata(h.fig,'lastTime',-inf);
end
