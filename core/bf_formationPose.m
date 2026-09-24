function [R,omega,s,vr]=bf_formationPose(P,V,cfg)
% Least-squares rigid fit to labelled vertices; valid for a noncollapsed array.
s=P-mean(P,2); vr=V-mean(V,2);
A=s*cfg.rho'; [U,D,W]=svd(A);
assert(D(2,2)>1e-7,'Formation collapsed or became collinear.');
R=U*diag([1 1 det(U*W')])*W';
% Differentiate the polar/Kabsch factor itself (not an approximate rigid
% velocity fit): K-K' = hat(omega_body)*S + S*hat(omega_body).
S=R'*A; S=(S+S')/2; K=R'*(vr*cfg.rho');
omega=R*((trace(S)*eye(3)-S)\bf_vee(K-K'));
end
