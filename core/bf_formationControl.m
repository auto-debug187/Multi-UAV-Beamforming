function [a,ref]=bf_formationControl(t,X,cfg)
%BF_FORMATIONCONTROL Outer-loop acceleration requests, in WORLD coordinates.
ref=bf_reference(t,cfg); P=X(1:3,:); V=X(4:6,:);
if strcmp(cfg.mode,'geometric')
    a=ref.a+cfg.control.Kp.*(ref.p-P)+cfg.control.Kd.*(ref.v-V);
    return
end
[R,wa,s,vr]=bf_formationPose(P,V,cfg);
E=R*ref.R'; e=bf_logSO3(E);
% Left-trivialized log derivative for E=R_actual*R_desired'.
ed=bf_leftJacInv(e)*(wa-E*ref.omega);
wc=ref.omega-cfg.control.beta*e;
wcd=ref.omegad-cfg.control.beta*ed;
pc=mean(P,2); vc=mean(V,2);
vcenter=ref.cv-cfg.control.kc*(pc-ref.c);
acenter=ref.ca-cfg.control.kc*(vc-ref.cv);
vd=zeros(3,cfg.N); vdd=vd;
for i=1:cfg.N
    vd(:,i)=vcenter+cross(wc,s(:,i));
    vdd(:,i)=acenter+cross(wcd,s(:,i))+cross(wc,vr(:,i));
end
[J,Jd,q]=bf_rigidity(P,V,cfg); v=V(:);
% Complete-graph constraint forces grow with N. Preserve the N=3 tuning.
shapeScale=3/cfg.N;
kv=cfg.control.kv*shapeScale; vf=vd(:)-kv*J'*q;
u=-cfg.control.ka*(v-vf)-cfg.control.kq*shapeScale*J'*q ...
    -kv*Jd'*q-2*kv*J'*J*v+vdd(:);
a=reshape(u,3,cfg.N);
% Explicit extension: damp out-of-plane deformation for N>3.
% Complete distance graphs at planar configurations lack first-order
% sensitivity to some out-of-plane motions. No unchanged paper theorem claimed.
if cfg.N>3
    n=R(:,3); nd=cross(wa,n);
    for i=1:cfg.N
        z=dot(n,s(:,i)); zd=dot(n,vr(:,i))+dot(nd,s(:,i));
        a(:,i)=a(:,i)-n*(cfg.control.kPlane*z+cfg.control.dPlane*zd);
    end
end
end
