function r=bf_reference(t,cfg)
%BF_REFERENCE Analytic smooth centroid and rigid-formation p,v,a references.
% Rotation gauge: projected fixed seed, not Euler azimuth/elevation.
w=2*pi/cfg.path.period; T=cfg.path.ramp;
if t<T
    u=max(0,t/T);
    phase=w*T*(2.5*u^4-3*u^5+u^6);
    pd=w*(10*u^3-15*u^4+6*u^5);
    pdd=w/T*(30*u^2-60*u^3+30*u^4);
else
    phase=w*(t-T/2); pd=w; pdd=0;
end
A=cfg.path.amplitude;
switch cfg.path.type
    case 'figure8'
        f=[A(1)*sin(phase); A(2)*sin(2*phase); A(3)*sin(phase)];
        fp=[A(1)*cos(phase); 2*A(2)*cos(2*phase); A(3)*cos(phase)];
        fpp=[-A(1)*sin(phase); -4*A(2)*sin(2*phase); -A(3)*sin(phase)];
    case 'circle'
        f=[A(1)*(cos(phase)-1); A(2)*sin(phase); 0];
        fp=[-A(1)*sin(phase); A(2)*cos(phase); 0];
        fpp=[-A(1)*cos(phase); -A(2)*sin(phase); 0];
    case 'hover'
        f=zeros(3,1); fp=f; fpp=f;
    otherwise
        error('Unknown path type.');
end
r.c=cfg.path.center+f; r.cv=fp*pd; r.ca=fpp*pd^2+fp*pdd;
[n,nd,ndd]=bf_unitDerivatives(cfg.receiver-r.c,-r.cv,-r.ca);
h=cfg.frameSeed; s=h-n*dot(n,h);
assert(norm(s)>0.05,'Formation frame seed nearly parallel to LOS; change frameSeed.');
sd=-nd*dot(n,h)-n*dot(nd,h);
sdd=-ndd*dot(n,h)-2*nd*dot(nd,h)-n*dot(ndd,h);
[b,bd,bdd]=bf_unitDerivatives(s,sd,sdd);
b2=cross(n,b); b2d=cross(nd,b)+cross(n,bd);
b2dd=cross(ndd,b)+2*cross(nd,bd)+cross(n,bdd);
r.R=[b b2 n]; r.Rd=[bd b2d nd]; r.Rdd=[bdd b2dd ndd];
W=r.Rd*r.R'; Wd=r.Rdd*r.R'+r.Rd*r.Rd';
r.omega=bf_vee((W-W')/2); r.omegad=bf_vee((Wd-Wd')/2);
r.n=n; r.p=r.c+r.R*cfg.rho;
r.v=r.cv+r.Rd*cfg.rho; r.a=r.ca+r.Rdd*cfg.rho;
end
