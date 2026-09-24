function [J,Jd,q]=bf_rigidity(P,V,cfg)
m=size(cfg.edges,1); J=zeros(m,3*cfg.N); Jd=J; q=zeros(m,1);
for k=1:m
    i=cfg.edges(k,1); j=cfg.edges(k,2);
    d=P(:,i)-P(:,j); dv=V(:,i)-V(:,j);
    ii=3*i-2:3*i; jj=3*j-2:3*j;
    J(k,ii)=d'; J(k,jj)=-d'; Jd(k,ii)=dv'; Jd(k,jj)=-dv';
    q(k)=dot(d,d)-cfg.edgeLength(k)^2;
end
end
