function v=bf_logSO3(R)
% Principal logarithm, with a robust branch close to pi.
c=max(-1,min(1,(trace(R)-1)/2)); t=acos(c);
if t<1e-7
    v=0.5*bf_vee(R-R');
elseif pi-t<1e-5
    [V,D]=eig((R+R')/2); [~,i]=max(real(diag(D)));
    axis=real(V(:,i)); axis=axis/norm(axis);
    s=bf_vee(R-R'); if dot(axis,s)<0, axis=-axis; end
    v=t*axis;
else
    v=t/(2*sin(t))*bf_vee(R-R');
end
end
