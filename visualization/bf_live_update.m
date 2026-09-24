function bf_live_update(h,t,X,cfg)
if ~isgraphics(h.fig), return; end
P=X(1:3,:); V=X(4:6,:); pc=mean(P,2); r=bf_reference(t,cfg);
[R,~,~]=bf_formationPose(P,V,cfg); n=R(:,3); los=cfg.receiver-pc;
theta=acosd(max(-1,min(1,dot(n,los/norm(los)))));
set(h.plane,'XData',P(1,:),'YData',P(2,:),'ZData',P(3,:));
Q=r.p(:,[1:cfg.N 1]); setLine(h.target,Q);
setLine(h.normal,[pc pc+n*min(4,norm(los))]); setLine(h.los,[pc cfg.receiver]);
setLine(h.centroid,pc);
for i=1:cfg.N
    addpoints(h.trace(i),P(1,i),P(2,i),P(3,i));
    Ri=bf_quatR(X(7:10,i)); rotor=P(:,i)+Ri*[cfg.vehicle.rotorXY;zeros(1,4)];
    setLine(h.arm(1,i),rotor(:,[1 3])); setLine(h.arm(2,i),rotor(:,[2 4]));
    set(h.label(i),'Position',P(:,i)'+[0 0 .15]);
end
de=zeros(size(cfg.edges,1),1);
for j=1:numel(de)
    de(j)=norm(P(:,cfg.edges(j,1))-P(:,cfg.edges(j,2)))-cfg.edgeLength(j);
end
addpoints(h.pointing,t,theta); addpoints(h.centerError,t,norm(pc-r.c));
addpoints(h.edgeError,t,sqrt(mean(de.^2)));
set(h.title,'String',sprintf('%s | N=%d | t=%.1f s | pointing %.2f deg',cfg.mode,cfg.N,t,theta));
drawnow limitrate;
end
function setLine(h,P)
set(h,'XData',P(1,:),'YData',P(2,:),'ZData',P(3,:));
end
